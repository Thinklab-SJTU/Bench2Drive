#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Corporation.
# authors: German Ros (german.ros@intel.com), Felipe Codevilla (felipe.alcm@gmail.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Challenge Evaluator Routes

Provisional code to evaluate Autonomous Agents for the CARLA Autonomous Driving challenge
"""
from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from distutils.version import LooseVersion
import importlib
import os
import pkg_resources
import sys
import carla
import signal

from srunner.scenariomanager.carla_data_provider import *
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.scenarios.scenario_manager import ScenarioManager
from leaderboard.scenarios.route_scenario import RouteScenario
from leaderboard.envs.sensor_interface import SensorConfigurationInvalid
from leaderboard.autoagents.agent_wrapper import AgentError, validate_sensor_configuration, TickRuntimeError
from leaderboard.utils.statistics_manager import StatisticsManager, FAILURE_MESSAGES
from leaderboard.utils.route_indexer import RouteIndexer
import atexit
import subprocess
import time
import random
from datetime import datetime

sensors_to_icons = {
    'sensor.camera.rgb':        'carla_camera',
    'sensor.lidar.ray_cast':    'carla_lidar',
    'sensor.other.radar':       'carla_radar',
    'sensor.other.gnss':        'carla_gnss',
    'sensor.other.imu':         'carla_imu',
    'sensor.opendrive_map':     'carla_opendrive_map',
    'sensor.speedometer':       'carla_speedometer'
}

import socket

def find_free_port(starting_port):
    port = starting_port
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("localhost", port))
                return port
        except OSError:
            port += 1

def get_weather_id(weather_conditions):
    from xml.etree import ElementTree as ET
    tree = ET.parse('leaderboard/data/weather.xml')
    root = tree.getroot()
    def conditions_match(weather, conditions):
        for (key, value) in weather:
            if key == 'route_percentage' : continue
            if str(getattr(conditions, key))!= value:
                return False
        return True
    for case in root.findall('case'):
        weather = case[0].items()
        if conditions_match(weather, weather_conditions):
            return case.items()[0][1]
    return None

class LeaderboardEvaluator(object):
    """
    Main class of the Leaderboard. Everything is handled from here,
    from parsing the given files, to preparing the simulation, to running the route.
    """

    # Tunable parameters
    client_timeout = 300.0  # in seconds
    frame_rate = 20.0      # in Hz

    def __init__(self, args, statistics_manager):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self.world = None
        self.manager = None
        self.sensors = None
        self.sensors_initialized = False
        self.sensor_icons = []
        self.agent_instance = None
        self.route_scenario = None

        self.statistics_manager = statistics_manager

        # This is the ROS1 bridge server instance. This is not encapsulated inside the ROS1 agent because the same
        # instance is used on all the routes (i.e., the server is not restarted between routes). This is done
        # to avoid reconnection issues between the server and the roslibpy client.
        self._ros1_server = None

        # Setup the simulation
        self.client, self.client_timeout, self.traffic_manager = self._setup_simulation(args)

        dist = pkg_resources.get_distribution("carla")
        if dist.version != 'leaderboard':
            if LooseVersion(dist.version) < LooseVersion('0.9.10'):
                raise ImportError("CARLA version 0.9.10.1 or newer required. CARLA version found: {}".format(dist))

        # Load agent
        module_name = os.path.basename(args.agent).split('.')[0]
        sys.path.insert(0, os.path.dirname(args.agent))
        self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(args.timeout, self.statistics_manager, args.debug)

        # Time control for summary purposes
        self._start_time = GameTime.get_time()
        self._end_time = None

        # Prepare the agent timer
        self._agent_watchdog = None
        signal.signal(signal.SIGINT, self._signal_handler)

        self._client_timed_out = False

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt.
        Either the agent initialization watchdog is triggered, or the runtime one at scenario manager
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError("Timeout: Agent took longer than {}s to setup".format(self.client_timeout))
        elif self.manager:
            self.manager.signal_handler(signum, frame)

    def __del__(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """
        if hasattr(self, 'manager') and self.manager:
            del self.manager
        if hasattr(self, 'world') and self.world:
            del self.world

    def _get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        if self._agent_watchdog:
            return self._agent_watchdog.get_status()
        return False

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        CarlaDataProvider.cleanup()

        if self._agent_watchdog:
            self._agent_watchdog.stop()

        try:
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
        except Exception as e:
            print("\n\033[91mFailed to stop the agent:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

        if self.route_scenario:
            self.route_scenario.remove_all_actors()
            self.route_scenario = None
            if self.statistics_manager:
                self.statistics_manager.remove_scenario()

        if self.manager:
            self._client_timed_out = not self.manager.get_running_status()
            self.manager.cleanup()

        # Make sure no sensors are left streaming
        alive_sensors = self.world.get_actors().filter('*sensor*')
        for sensor in alive_sensors:
            sensor.stop()
            sensor.destroy()

    def _setup_simulation(self, args):
        """
        Prepares the simulation by getting the client, and setting up the world and traffic manager settings
        """
        self.carla_path = os.environ["CARLA_ROOT"]
        args.port = find_free_port(args.port)
        cmd1 = f"{os.path.join(self.carla_path, 'CarlaUE4.sh')} -RenderOffScreen -nosound -carla-rpc-port={args.port} -graphicsadapter={args.gpu_rank}"
        self.server = subprocess.Popen(cmd1, shell=True, preexec_fn=os.setsid)
        print(cmd1, self.server.returncode, flush=True)
        atexit.register(os.killpg, self.server.pid, signal.SIGKILL)
        time.sleep(30)
            
        attempts = 0
        num_max_restarts = 20
        while attempts < num_max_restarts:
            try:
                client = carla.Client(args.host, args.port)
                if args.timeout:
                    client_timeout = args.timeout
                client.set_timeout(client_timeout)

                settings = carla.WorldSettings(
                    synchronous_mode = True,
                    fixed_delta_seconds = 1.0 / self.frame_rate,
                    deterministic_ragdolls = True,
                    spectator_as_ego = False
                )
                client.get_world().apply_settings(settings)
                print(f"load_world success , attempts={attempts}", flush=True)
                break
            except Exception as e:
                print(f"load_world failed , attempts={attempts}", flush=True)
                print(e, flush=True)
                attempts += 1
                time.sleep(5)
        attempts = 0
        num_max_restarts = 40
        while attempts < num_max_restarts:
            try:
                args.traffic_manager_port = find_free_port(args.traffic_manager_port)
                traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
                traffic_manager.set_synchronous_mode(True)
                traffic_manager.set_hybrid_physics_mode(True)
                print(f"traffic_manager init success, try_time={attempts}", flush=True)
                break
            except Exception as e:
                print(f"traffic_manager init fail, try_time={attempts}", flush=True)
                print(e, flush=True)
                attempts += 1
                time.sleep(5)
        return client, client_timeout, traffic_manager

    def _reset_world_settings(self):
        """
        Changes the modified world settings back to asynchronous
        """
        # Has simulation failed?
        if self.world and self.manager and not self._client_timed_out:
            # Reset to asynchronous mode
            self.world.tick()  # TODO: Make sure all scenario actors have been destroyed
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            settings.deterministic_ragdolls = False
            settings.spectator_as_ego = True
            self.world.apply_settings(settings)

            # Make the TM back to async
            self.traffic_manager.set_synchronous_mode(False)
            self.traffic_manager.set_hybrid_physics_mode(False)

    def _load_and_wait_for_world(self, args, town):
        """
        Load a new CARLA world without changing the settings and provide data to CarlaDataProvider
        """
        self.world = self.client.load_world(town, reset_settings=False)

        # Large Map settings are always reset, for some reason
        settings = self.world.get_settings()
        settings.tile_stream_distance = 650
        settings.actor_active_distance = 650
        self.world.apply_settings(settings)

        self.world.reset_all_traffic_lights()
        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_traffic_manager_port(args.traffic_manager_port)
        CarlaDataProvider.set_world(self.world)

        # This must be here so that all route repetitions use the same 'unmodified' seed
        self.traffic_manager.set_random_device_seed(args.traffic_manager_seed)

        # Wait for the world to be ready
        self.world.tick()

        map_name = CarlaDataProvider.get_map().name.split("/")[-1]
        if map_name != town:
            raise Exception("The CARLA server uses the wrong map!"
                            " This scenario requires the use of map {}".format(town))

    def _register_statistics(self, route_index, entry_status, crash_message=""):
        """
        Computes and saves the route statistics
        """
        print("\033[1m> Registering the route statistics\033[0m", flush=True)
        self.statistics_manager.save_entry_status(entry_status)
        self.statistics_manager.compute_route_statistics(
            route_index, self.manager.scenario_duration_system, self.manager.scenario_duration_game, crash_message
        )

    def _load_and_run_scenario(self, args, config):
        """
        Load and run the scenario given by config.

        Depending on what code fails, the simulation will either stop the route and
        continue from the next one, or report a crash and stop.
        """
        crash_message = ""
        entry_status = "Started"

        print("\n\033[1m========= Preparing {} (repetition {}) =========\033[0m".format(config.name, config.repetition_index), flush=True)

        # Prepare the statistics of the route
        route_name = f"{config.name}_rep{config.repetition_index}"
        scenario_name = config.scenario_configs[0].name
        town_name = str(config.town)
        weather_id = get_weather_id(config.weather[0][1])
        currentDateAndTime = datetime.now()
        currentTime = currentDateAndTime.strftime("%m_%d_%H_%M_%S")
        save_name = f"{route_name}_{town_name}_{scenario_name}_{weather_id}_{currentTime}"
        self.statistics_manager.create_route_data(route_name, scenario_name, weather_id, save_name, town_name, config.index)

        print("\033[1m> Loading the world\033[0m", flush=True)

        # Load the world and the scenario
        try:
            self._load_and_wait_for_world(args, config.town)
            self.route_scenario = RouteScenario(world=self.world, config=config, debug_mode=args.debug)
            self.statistics_manager.set_scenario(self.route_scenario)

        except Exception:
            # The scenario is wrong -> set the ejecution to crashed and stop
            print("\n\033[91mThe scenario could not be loaded:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Simulation"]
            self._register_statistics(config.index, entry_status, crash_message)
            self._cleanup()
            return True

        print("\033[1m> Setting up the agent\033[0m", flush=True)

        # Set up the user's agent, and the timer to avoid freezing the simulation
        try:
            self._agent_watchdog = Watchdog(args.timeout)
            self._agent_watchdog.start()
            agent_class_name = getattr(self.module_agent, 'get_entry_point')()
            agent_class_obj = getattr(self.module_agent, agent_class_name)

            # Start the ROS1 bridge server only for ROS1 based agents.
            if getattr(agent_class_obj, 'get_ros_version')() == 1 and self._ros1_server is None:
                from leaderboard.autoagents.ros1_agent import ROS1Server
                self._ros1_server = ROS1Server()
                self._ros1_server.start()

            self.agent_instance = agent_class_obj(args.host, args.port, args.debug)
            self.agent_instance.set_global_plan(self.route_scenario.gps_route, self.route_scenario.route)
            args.agent_config = args.agent_config + '+' + save_name
            self.agent_instance.setup(args.agent_config)

            # Check and store the sensors
            if not self.sensors:
                self.sensors = self.agent_instance.sensors()
                track = self.agent_instance.track

                validate_sensor_configuration(self.sensors, track, args.track)

                self.sensor_icons = [sensors_to_icons[sensor['type']] for sensor in self.sensors]
                self.statistics_manager.save_sensors(self.sensor_icons)
                self.statistics_manager.write_statistics()

                self.sensors_initialized = True

            self._agent_watchdog.stop()
            self._agent_watchdog = None

        except SensorConfigurationInvalid as e:
            # The sensors are invalid -> set the ejecution to rejected and stop
            print("\n\033[91mThe sensor's configuration used is invalid:", flush=True)
            print(f"{e}\033[0m\n", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Sensors"]
            self._register_statistics(config.index, entry_status, crash_message)
            self._cleanup()
            return True

        except Exception as e:
            # The agent setup has failed -> start the next route
            print("\n\033[91mCould not set up the required agent:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)
            print(f"{e}\033[0m\n", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Agent_init"]
            self._register_statistics(config.index, entry_status, crash_message)
            self._cleanup()
            return True

        print("\033[1m> Running the route\033[0m", flush=True)

        # Run the scenario
        try:
            # Load scenario and run it
            if args.record:
                self.client.start_recorder("{}/{}_rep{}.log".format(args.record, config.name, config.repetition_index))
            self.manager.load_scenario(self.route_scenario, self.agent_instance, config.index, config.repetition_index)
            self.manager.tick_count = 0
            self.manager.run_scenario()

        except AgentError:
            # The agent has failed -> stop the route
            print("\n\033[91mStopping the route, the agent has crashed:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m")

            entry_status, crash_message = FAILURE_MESSAGES["Agent_runtime"]

        except KeyboardInterrupt:
            return True
        
        except TickRuntimeError:
            entry_status, crash_message = "Started", "TickRuntime"
        
        except Exception:
            print("\n\033[91mError during the simulation:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Simulation"]

        # Stop the scenario
        try:
            print("\033[1m> Stopping the route\033[0m", flush=True)
            self.manager.stop_scenario()
            self._register_statistics(config.index, entry_status, crash_message)

            if args.record:
                self.client.stop_recorder()

            self._cleanup()

        except Exception:
            print("\n\033[91mFailed to stop the scenario, the statistics might be empty:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

            _, crash_message = FAILURE_MESSAGES["Simulation"]

        # If the simulation crashed, stop the leaderboard, for the rest, move to the next route
        return crash_message == "Simulation crashed"

    def run(self, args):
        """
        Run the challenge mode
        """
        route_indexer = RouteIndexer(args.routes, args.repetitions, args.routes_subset)

        if args.resume:
            resume = route_indexer.validate_and_resume(args.checkpoint)
        else:
            resume = False

        if resume:
            self.statistics_manager.add_file_records(args.checkpoint)
        else:
            self.statistics_manager.clear_records()
        self.statistics_manager.save_progress(route_indexer.index, route_indexer.total)
        self.statistics_manager.write_statistics()

        crashed = False
        t1 = time.time()
        while route_indexer.peek() and not crashed:

            # Run the scenario
            config = route_indexer.get_next_config()
            crashed = self._load_and_run_scenario(args, config)
            print(crashed, flush=True)
            # Save the progress and write the route statistics
            self.statistics_manager.save_progress(route_indexer.index, route_indexer.total)
            self.statistics_manager.write_statistics()
            if crashed:
                print(f'{route_indexer.index} crash, [{route_indexer.index}/{route_indexer.total}], please restart', flush=True)
                break

        # Shutdown ROS1 bridge server if necessary
        if self._ros1_server is not None:
            self._ros1_server.shutdown()

        # Go back to asynchronous mode
        self._reset_world_settings()

        if not crashed:
            # Save global statistics
            print(f"cost time={time.time()-t1}", flush=True)
            print("\033[1m> Registering the global statistics\033[0m", flush=True)
            self.statistics_manager.compute_global_statistics()
            self.statistics_manager.validate_and_write_statistics(self.sensors_initialized, crashed)
        
        if crashed:
            cmd2 = "ps -ef | grep '-graphicsadapter="+ str(args.gpu_rank) + "' | grep -v grep | awk '{print $2}' | xargs -r kill -9"
            server = subprocess.Popen(cmd2, shell=True, preexec_fn=os.setsid)
            atexit.register(os.killpg, server.pid, signal.SIGKILL)

        return crashed

def main():
    description = "CARLA AD Leaderboard Evaluation: evaluate your Agent in CARLA scenarios\n"

    # general parameters
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host', default='localhost',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default=2000, type=int,
                        help='TCP port to listen to (default: 2000)')
    parser.add_argument('--traffic-manager-port', default=8000, type=int,
                        help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--traffic-manager-seed', default=0, type=int,
                        help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--debug', type=int,
                        help='Run with debug output', default=0)
    parser.add_argument('--record', type=str, default='',
                        help='Use CARLA recording feature to create a recording of the scenario')
    parser.add_argument('--timeout', default=600.0, type=float,
                        help='Set the CARLA client timeout value in seconds')

    # simulation setup
    parser.add_argument('--routes', required=True,
                        help='Name of the routes file to be executed.')
    parser.add_argument('--routes-subset', default='', type=str,
                        help='Execute a specific set of routes')
    parser.add_argument('--repetitions', type=int, default=1,
                        help='Number of repetitions per route.')

    # agent-related options
    parser.add_argument("-a", "--agent", type=str,
                        help="Path to Agent's py file to evaluate", required=True)
    parser.add_argument("--agent-config", type=str,
                        help="Path to Agent's configuration file", default="")

    parser.add_argument("--track", type=str, default='SENSORS',
                        help="Participation track: SENSORS, MAP")
    parser.add_argument('--resume', type=bool, default=False,
                        help='Resume execution from last checkpoint?')
    parser.add_argument("--checkpoint", type=str, default='./simulation_results.json',
                        help="Path to checkpoint used for saving statistics and resuming")
    parser.add_argument("--debug-checkpoint", type=str, default='./live_results.txt',
                        help="Path to checkpoint used for saving live results")
    parser.add_argument("--gpu-rank", type=int, default=0)
    arguments = parser.parse_args()

    statistics_manager = StatisticsManager(arguments.checkpoint, arguments.debug_checkpoint)
    leaderboard_evaluator = LeaderboardEvaluator(arguments, statistics_manager)
    crashed = leaderboard_evaluator.run(arguments)

    del leaderboard_evaluator

    if crashed:
        sys.exit(-1)
    else:
        sys.exit(0)

if __name__ == '__main__':
    main()
