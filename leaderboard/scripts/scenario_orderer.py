#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
from lxml import etree
import os

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner

SCENARIO_TYPES = {

    # Junction scenarios
    "SignalizedJunctionLeftTurn": [
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "SignalizedJunctionRightTurn": [
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "OppositeVehicleRunningRedLight": [
        ["direction", "choice"],
    ],
    "NonSignalizedJunctionLeftTurn": [
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "NonSignalizedJunctionRightTurn": [
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "OppositeVehicleTakingPriority": [
        ["direction", "choice"],
    ],

    # Crossing actors
    "DynamicObjectCrossing": [
        ["distance", "value"],
        ["direction", "value"],
        ["blocker_model", "value"],
        ["crossing_angle", "value"]
    ],
    "ParkingCrossingPedestrian": [
        ["distance", "value"],
        ["direction", "choice"],
        ["crossing_angle", "value"],
    ],
    "PedestrianCrossing": [
    ],
    "VehicleTurningRoute": [
    ],
    "VehicleTurningRoutePedestrian": [
    ],
    "BlockedIntersection": [
    ],

    # Actor flows
    "EnterActorFlow": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "EnterActorFlowV2": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "InterurbanActorFlow": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "InterurbanAdvancedActorFlow": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "HighwayExit": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "MergerIntoSlowTraffic": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "MergerIntoSlowTrafficV2": [
        ["start_actor_flow", "location driving"],
        ["end_actor_flow", "location driving"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],
    "CrossingBicycleFlow": [
        ["start_actor_flow", "location bicycle"],
        ["flow_speed", "value"],
        ["source_dist_interval", "interval"],
    ],

    # Route obstacles
    "ConstructionObstacle": [
        ["distance", "value"],
        ["direction", "value"],
        ["speed", "value"],
    ],
    "ConstructionObstacleTwoWays": [
        ["distance", "value"],
        ["frequency", "interval"],
    ],
    "Accident": [
        ["distance", "value"],
        ["direction", "value"],
        ["speed", "value"],
    ],
    "AccidentTwoWays": [
        ["distance", "value"],
        ["frequency", "interval"],
    ],
    "ParkedObstacle": [
        ["distance", "value"],
        ["direction", "value"],
        ["speed", "value"],
    ],
    "ParkedObstacleTwoWays": [
        ["distance", "value"],
        ["frequency", "interval"],
    ],
    "VehicleOpensDoorTwoWays": [
        ["distance", "value"],
        ["frequency", "interval"],
    ],
    "HazardAtSideLane": [
        ["distance", "value"],
        ["speed", "value"],
        ["bicycle_drive_distance", "value"],
        ["bicycle_speed", "value"],
    ],
    "HazardAtSideLaneTwoWays": [
        ["distance", "value"],
        ["frequency", "value"],
        ["bicycle_drive_distance", "value"],
        ["bicycle_speed", "value"],
    ],
    "InvadingTurn": [
        ["distance", "value"],
        ["offset", "value"],
    ],

    # Cut ins
    "HighwayCutIn": [
        ["other_actor_location", "location driving"],
    ],
    "ParkingCutIn": [
        ["direction", "choice"],
    ],
    "StaticCutIn": [
        ["distance", "value"],
        ["direction", "choice"],
    ],

    # Others
    "ControlLoss": [
    ],
    "HardBreakRoute": [
    ],
    "ParkingExit": [
        ["direction", "choice"],
        ["front_vehicle_distance", "value"],
        ["behind_vehicle_distance", "value"],
    ],
    "YieldToEmergencyVehicle": [
        ["distance", "value"],
    ],

    # Special ones
    "BackgroundActivityParametrizer": [
        ["num_front_vehicles", "value"],
        ["num_back_vehicles", "value"],
        ["road_spawn_dist", "value"],
        ["opposite_source_dist", "value"],
        ["opposite_max_actors", "value"],
        ["opposite_spawn_dist", "value"],
        ["opposite_active", "bool"],
        ["junction_source_dist", "value"],
        ["junction_max_actors", "value"],
        ["junction_spawn_dist", "value"],
        ["junction_source_perc", "value"],
    ],
    "PriorityAtJunction": [
    ],

}

MAPS_LOCATIONS = {
    "Town01": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01.xodr",
    "Town01_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01_Opt.xodr",
    "Town02": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town02.xodr",
    "Town02_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town02_Opt.xodr",
    "Town03": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr",
    "Town03_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town04_Opt.xodr",
    "Town04": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town04.xodr",
    "Town04_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town04_Opt.xodr",
    "Town05": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr",
    "Town05_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05_Opt.xodr",
    "Town06": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town06.xodr",
    "Town06_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town06_Opt.xodr",
    "Town07": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town07.xodr",
    "Town07_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town07_Opt.xodr",
    "Town10HD": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr",
    "Town10HD_Opt": "Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr",
    "Town11": "Unreal/CarlaUE4/Content/Carla/Maps/Town11/OpenDrive/Town11.xodr",
    "Town12": "Unreal/CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr",
    "Town13": "Unreal/CarlaUE4/Content/Carla/Maps/Town13/OpenDrive/Town13.xodr",
}

def order_saved_scenarios(filename, route_id, tmap, grp):
    def convert_elem_to_location(elem):
        """Convert an ElementTree.Element to a CARLA Location"""
        return carla.Location(float(elem.attrib.get('x')), float(elem.attrib.get('y')), float(elem.attrib.get('z')))

    def get_scenario_route_position(trigger_location):
        position = 0
        distance = float('inf')
        for i, (wp, _) in enumerate(route_wps):
            route_distance = wp.transform.location.distance(trigger_location)
            if route_distance < distance:
                distance = route_distance
                position = i
        return position

    def indent(elem, spaces=3, level=0):
        i = "\n" + level * spaces * " "
        j = "\n" + (level + 1) * spaces * " "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = j
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for subelem in elem:
                indent(subelem, spaces, level+1)
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i
        return elem

    tree = etree.parse(filename)
    root = tree.getroot()

    scenarios = None
    route_wps = []
    prev_route_keypoint = None
    scenarios = []

    for route in root.iter("route"):
        if route.attrib['id'] != route_id:
            continue

        # Scenarios data
        for scenario in route.find('scenarios').iter('scenario'):
            scenarios.append(scenario)

        # Route data
        for position in route.find('waypoints').iter('position'):
            route_keypoint = convert_elem_to_location(position)
            if prev_route_keypoint:
                route_wps.extend(grp.trace_route(prev_route_keypoint, route_keypoint))
            prev_route_keypoint = route_keypoint

    if scenarios is None:
        print(f"\n\033[91mCouldn't find the id '{route_id} in the given routes file\033[0m")
        return

    # Order the scenarios according to route position
    scenario_and_pos = []
    for scenario in scenarios:
        trigger_location = convert_elem_to_location(scenario.find('trigger_point'))
        route_position = get_scenario_route_position(trigger_location)
        scenario_and_pos.append([scenario, route_position])
    scenario_and_pos = sorted(scenario_and_pos, key=lambda x: x[1])

    # Update the scenarios. TODO: reorder them
    scenario_names = {}
    for scen_type in list(SCENARIO_TYPES):
        scenario_names[scen_type] = 1

    for scenario, _ in scenario_and_pos:
        scen_type = scenario.attrib['type']
        scen_name = f"{scen_type}_{scenario_names[scen_type]}"
        scenario.set("name", scen_name)
        scenario_names[scen_type] += 1

    # Prettify the xml. A bit of automatic indentation, a bit of manual one
    spaces = 3
    indent(tree.getroot(), spaces)
    tree.write(filename)

    with open(filename, 'r') as f:
        data = f.read()
    temp = data.replace("   </", "</")  # The 'indent' function fails for these cases

    weather_spaces = spaces*4*" "
    temp = temp.replace(" cloudiness", "\n" + weather_spaces + "cloudiness")
    temp = temp.replace(" wind_intensity", "\n" + weather_spaces + "wind_intensity")
    temp = temp.replace(" fog_density", "\n" + weather_spaces + "fog_density")
    new_data = temp.replace(" mie_scattering_scale", "\n" + weather_spaces + "mie_scattering_scale")

    with open(filename, 'w') as f:
        f.write(new_data)

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', required=True, help='File at which to place the scenarios')
    args = argparser.parse_args()

    root = etree.parse(args.file).getroot()
    prev_town = None
    for route in root.iter("route"):

        route_id = route.attrib['id']
        route_town = route.attrib['town']

        if route_town not in MAPS_LOCATIONS:
            print(f"Ignoring route '{route_id}' as it uses an unknown map '{route_town}")
            continue
        elif route_town != prev_town:
            full_name = os.environ["CARLA_ROOT"] + "/" + MAPS_LOCATIONS[route_town]
            with open(full_name, 'r') as f:
                map_contents = f.read()
            tmap = carla.Map(route_town, map_contents)
            grp = GlobalRoutePlanner(tmap, 1.0)

        print(f"Parsing route '{route_id}'")

    file_path = args.file[0]
    route_id = args.file[1] if len(args.file) > 1 else 0

    order_saved_scenarios(file_path, route_id, tmap)
