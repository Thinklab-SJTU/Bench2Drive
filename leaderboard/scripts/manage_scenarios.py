import argparse
from argparse import RawTextHelpFormatter
import math

import carla

from leaderboard.utils.checkpoint_tools import fetch_dict
from leaderboard.utils.route_parser import DIST_THRESHOLD

SCENARIO_COLOR = {
    "Scenario1": [carla.Color(255, 0, 0), "Red"],
    "Scenario2": [carla.Color(0, 255, 0), "Green"],
    "Scenario3": [carla.Color(0, 0, 255), "Blue"],
    "Scenario4": [carla.Color(255, 100, 0), "Orange"],
    "Scenario5": [carla.Color(0, 255, 100), "Blueish green"],
    "Scenario6": [carla.Color(100, 0, 255), "Purple"],
    "Scenario7": [carla.Color(255, 100, 255), "Pink"],
    "Scenario8": [carla.Color(255, 255, 100), "Yellow"],
    "Scenario9": [carla.Color(100, 255, 255), "Light Blue"], 
    "Scenario10": [carla.Color(100, 100, 100), "Gray"]
}

DEBUG_HEIGHT_INTERVAL = 0.3

def get_scenario_transform(scenario_dict):
    return carla.Transform(
        carla.Location(
            float(scenario_dict['transform']['x']),
            float(scenario_dict['transform']['y']),
            float(scenario_dict['transform']['z'])
        ),
        carla.Rotation(
            float(scenario_dict['transform']['pitch']),
            float(scenario_dict['transform']['yaw']),
            0
        )
    )

def get_color_validity(waypoint_transform, scenario_transform, scenario_type, scenario_index, debug):
    """
    Uses the same condition as in route_scenario to see if they will
    be differentiated
    """
    ANGLE_THRESHOLD = 10

    dx = float(waypoint_transform.location.x) - scenario_transform.location.x
    dy = float(waypoint_transform.location.y) - scenario_transform.location.y
    dz = float(waypoint_transform.location.z) - scenario_transform.location.z
    dpos = math.sqrt(dx * dx + dy * dy + dz * dz)
    dyaw = (float(waypoint_transform.rotation.yaw) - scenario_transform.rotation.yaw) % 360

    if dpos > DIST_THRESHOLD:
        if not debug:
            print("WARNING: Found a scenario with the wrong position "
                  "(Type: {}, Index: {})".format(scenario_type, scenario_index))
        return carla.Color(255, 0, 0)
    if dyaw > ANGLE_THRESHOLD and dyaw < (360 - ANGLE_THRESHOLD):
        if not debug:
            print("WARNING: Found a scenario with the wrong orientation "
                  "(Type: {}, Index: {})".format(scenario_type, scenario_index))
        return carla.Color(0, 0, 255)
    return carla.Color(0, 255, 0)


def create_scenarios(world, town_data, tmap, scenarios, debug):
    """
    Creates new S7 to S10 by moving 5 meters backwards from an already existing S4.
    They have to be manually added to the json file but information is given to simplify that process
    """
    for scenario_data in town_data:
        # Get the desired scenario data
        scenario_type = scenario_data["scenario_type"]
        if scenario_type not in scenarios:
            continue

        number = float(scenario_type[8:])
        debug_loc_height = carla.Location(z=DEBUG_HEIGHT_INTERVAL * number)
        debug_str_height = carla.Location(z=DEBUG_HEIGHT_INTERVAL * number + 0.1)
        color = SCENARIO_COLOR[scenario_type][0]

        scenario_list = scenario_data["available_event_configurations"]
        for i in range(len(scenario_list)):
            # Get the individual scenario data
            scenario_dict = scenario_list[i]
            scenario_transform = get_scenario_transform(scenario_dict)
            scenario_location = scenario_transform.location
            world.debug.draw_point(
                scenario_location + debug_loc_height, float(0.15), color)
            world.debug.draw_string(
                scenario_location + debug_str_height , str(i), False, carla.Color(0, 0, 0), 1000)

            # Get the new scenario data
            scenario_wp = tmap.get_waypoint(scenario_location)
            new_transform = scenario_wp.previous(5)[0].transform
            new_location = new_transform.location
            world.debug.draw_point(
                new_location + debug_loc_height, float(0.15), carla.Color(0,0,0))
            world.debug.draw_string(
                new_location + debug_str_height , str(i), False, carla.Color(0, 0, 0), 1000)

            if debug:
                spectator = world.get_spectator()
                spectator.set_transform(carla.Transform(new_location + carla.Location(z=50),
                                                        carla.Rotation(pitch=-90)))
                input(" New Scenario [{}/{}] at (x={}, y={}, z={}, yaw={}). Press Enter to continue".format(
                    i, len(scenario_list) -1,
                    round(new_location.x, 1),
                    round(new_location.y, 1),
                    round(new_location.z, 1),
                    round(new_transform.rotation.yaw, 1))
                )

        world.wait_for_tick()


def validate_scenarios(world, town_data, tmap, scenarios, debug):
    """
    Validates that all the scenarios can be triggered (by having a waypoint closeby). Color code
    - Can be triggered: Green
    - Didn't pass the position check: Red
    - Didn't pass the yaw check: Blue
    """
    for scenario_data in town_data:
        # Get the desired scenario data
        scenario_type = scenario_data["scenario_type"]
        if scenario_type not in scenarios:
            continue

        number = float(scenario_type[8:])
        debug_loc_height = carla.Location(z=DEBUG_HEIGHT_INTERVAL * number)
        debug_str_height = carla.Location(z=DEBUG_HEIGHT_INTERVAL * number + 0.1)

        scenario_list = scenario_data["available_event_configurations"]
        for i in range(len(scenario_list)):
            # Get the individual scenario data
            scenario_dict = scenario_list[i]
            scenario_transform = get_scenario_transform(scenario_dict)
            scenario_location = scenario_transform.location

            # Check if the scenario is close enough to the wp
            scenario_wp = tmap.get_waypoint(scenario_location)
            color = get_color_validity(scenario_wp.transform, scenario_transform, scenario_type, i, debug)

            world.debug.draw_point(
                scenario_location + debug_loc_height, float(0.15), color)
            world.debug.draw_string(
                scenario_location + debug_str_height , str(i), False, carla.Color(0, 0, 0), 1000)

            if debug:
                spectator = world.get_spectator()
                spectator.set_transform(carla.Transform(scenario_location + carla.Location(z=50),
                                                        carla.Rotation(pitch=-90)))
                input(" Scenario [{}/{}] at (x={}, y={}, z={}, yaw={}). Press Enter to continue".format(
                    i, len(scenario_list) - 1,
                    round(scenario_location.x, 1),
                    round(scenario_location.y, 1),
                    round(scenario_location.z, 1),
                    round(scenario_transform.rotation.yaw, 1))
                )

        world.wait_for_tick()

def draw_scenarios(world, town_data, scenarios, debug):
    """
    Draws all the scenario trigger positions. Each scenario is of a specific color
    """
    final_message = "\n------------------------------\n"
    for scenario_data in town_data:
        # Get the desired scenario data
        scenario_type = scenario_data["scenario_type"]
        if scenario_type not in scenarios:
            continue

        number = float(scenario_type[8:])
        debug_loc_height = carla.Location(z=DEBUG_HEIGHT_INTERVAL * number)
        debug_str_height = carla.Location(z=DEBUG_HEIGHT_INTERVAL * number + 0.1)
        color = SCENARIO_COLOR[scenario_type][0]
        end_color= "\x1b[0m"
        true_color = "\x1b[38;2;" + str(color.r) +";" + str(color.g) + ";" + str(color.b) + "m"
        final_message += "{}{} is colored as {}{}\n".format(true_color, scenario_type, SCENARIO_COLOR[scenario_type][1], end_color)

        scenario_list = scenario_data["available_event_configurations"]
        for i in range(len(scenario_list)):
            # Get the individual scenario data
            scenario_dict = scenario_list[i]
            scenario_transform = get_scenario_transform(scenario_dict)
            scenario_location = scenario_transform.location
            world.debug.draw_point(
                scenario_location + debug_loc_height, float(0.15), color)
            world.debug.draw_string(
                scenario_location + debug_str_height , str(i), False, carla.Color(0, 0, 0), 1000)

            if debug:
                spectator = world.get_spectator()
                spectator.set_transform(carla.Transform(scenario_location + carla.Location(z=50),
                                                        carla.Rotation(pitch=-90)))
                input(" Scenario [{}/{}] at (x={}, y={}, z={}, yaw={}). Press Enter to continue".format(
                    i, len(scenario_list) - 1,
                    round(scenario_location.x, 1),
                    round(scenario_location.y, 1),
                    round(scenario_location.z, 1),
                    round(scenario_transform.rotation.yaw, 1))
                )

        world.wait_for_tick()
    final_message += "------------------------------\n"

def main():
    """Used to help with the visualization of the scenario trigger points"""
    # general parameters
    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter,
                                     description="Draw, validate or create Leaderboard scenarios. "
                                                 "For multiple scenarios, separate them by spaces (example '1 3 6 9')")
    parser.add_argument('--host', default='localhost',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default='2000',
                        help='TCP port to listen to (default: 2000)')

    # Path from which all scenarios will be taken
    parser.add_argument('-f', '--file-path', default="../data/all_towns_traffic_scenarios_public.json",
                        help='path to the .json file containing the scenarios')

    # Different tests. CHOOSE ONLY ONE.
    parser.add_argument('--draw-scenarios', nargs='+',
                        help='Draw the scenarios.')
    parser.add_argument('--validate-scenarios', nargs='+',
                        help='Checks whether or not a scenario is well positioned in lane')
    parser.add_argument('--create-junction-scenarios', action='store_true',
                        help='Creates scenarios 7 to 10 using scenario 4 as reference')

    # Debug tool, to easily differentiate between scenarios
    parser.add_argument('--debug', action='store_true',
                        help='Scenarios are printed one by one, and additional information is given')

    # Town loading arguments
    parser.add_argument('--load-town',
                        help='Loads a specific town (example "Town01")')
    parser.add_argument('--reload-town', action='store_true',
                        help='Reloads the town')
    args = parser.parse_args()

    if args.load_town and args.reload_town:
        raise ValueError("'load_town' and 'reload' can't be active at the same time")

    try:
        client = carla.Client(args.host, int(args.port))
        client.set_timeout(20)
        if args.load_town:
            world = client.load_world(args.load_town)
        elif args.reload_town:
            world = client.reload_world()
        else:
            world = client.get_world()
        tmap = world.get_map()

        settings = world.get_settings()
        settings.fixed_delta_seconds = None
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # Read the json file
        data = fetch_dict(args.file_path)
        town_name = world.get_map().name.split("/")[-1]
        town_data = data["available_scenarios"][0][town_name]

        if args.draw_scenarios:
            scenarios = ["Scenario" + ar_sc for ar_sc in args.draw_scenarios]
            draw_scenarios(world, town_data, scenarios, args.debug)
        elif args.validate_scenarios:
            scenarios = ["Scenario" + ar_sc for ar_sc in args.validate_scenarios]
            validate_scenarios(world, town_data, tmap, scenarios, args.debug)
        elif args.create_junction_scenarios:
            scenarios = ["Scenario4"]
            create_scenarios(world, town_data, tmap, scenarios, args.debug)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
