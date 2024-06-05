#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
from lxml import etree
import sys

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner

LIFE_TIME = 10000

SCENARIO_TYPES ={

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

def show_saved_scenarios(filename, route_id, world):
    def convert_elem_to_location(elem):
        """Convert an ElementTree.Element to a CARLA Location"""
        return carla.Location(float(elem.attrib.get('x')), float(elem.attrib.get('y')), float(elem.attrib.get('z')))

    tree = etree.parse(filename)
    root = tree.getroot()

    found_id = False

    for route in root.iter("route"):
        if route.attrib['id'] != route_id:
            continue

        found_id = True

        for scenario in route.find('scenarios').iter('scenario'):
            name = scenario.attrib.get('name')
            trigger_location = convert_elem_to_location(scenario.find('trigger_point'))
            world.debug.draw_point(trigger_location + carla.Location(z=0.2), size=0.1, color=carla.Color(125, 0, 0))
            world.debug.draw_string(trigger_location + carla.Location(z=0.5), name, True, color=carla.Color(0, 0 , 125), life_time=LIFE_TIME)

    if not found_id:
        print(f"\n\033[91mCouldn't find the id '{route_id} in the given routes file\033[0m")
        return

def get_scenario_type(tmap, world, spectator):
    while True:
        scen_type = input("\033[1m> Specify the scenario type \033[0m")
        if scen_type not in list(SCENARIO_TYPES):
            print(f"\033[1m\033[93mScenario type '{scen_type}' doesn't match any of the know scenarios\033[0m")
        else:
            break

    wp = tmap.get_waypoint(spectator.get_location())
    world.debug.draw_point(wp.transform.location + carla.Location(z=0.2), size=0.1, color=carla.Color(125, 0, 0))
    world.debug.draw_string(wp.transform.location + carla.Location(z=0.5), scen_type, True, color=carla.Color(0, 0 , 125), life_time=LIFE_TIME)
    trigger_point = (
        str(round(wp.transform.location.x, 1)),
        str(round(wp.transform.location.y, 1)),
        str(round(wp.transform.location.z, 1)),
        str(round(wp.transform.rotation.yaw, 1))
    )
    return scen_type, trigger_point

def get_attributes_data(scen_type, trigger_point, tmap, world, spectator):
    attribute_list = SCENARIO_TYPES[scen_type]
    scenario_attributes = [['trigger_point', 'transform', trigger_point]]
    for attribute in attribute_list:
        a_name, a_type = attribute
        if a_type == 'transform':
            a_data = get_transform_data(a_name, scen_type, tmap, world, spectator)
        elif 'location' in a_type:
            a_data = get_location_data(a_name, scen_type, tmap, world, spectator, a_type)
        elif a_type in ('value', 'choice', 'bool'):
            a_data = get_value_data(a_name)
        elif a_type == 'interval':
            a_data = get_interval_data(a_name)
        else:
            raise ValueError("Unknown attribute type")

        if a_data:  # Ignore the attributes that use default values
            scenario_attributes.append([a_name, a_type, a_data])
    return scenario_attributes

def get_transform_data(a_name, scen_type, tmap, world, spectator):
    input(f"\033[1m> Enter the '{a_name}' transform \033[0m")
    wp = tmap.get_waypoint(spectator.get_location())
    world.debug.draw_point(wp.transform.location + carla.Location(z=0.2), size=0.1, color=carla.Color(125, 0, 0))
    world.debug.draw_string(wp.transform.location + carla.Location(z=0.5), scen_type, True, color=carla.Color(0, 0 , 125), life_time=LIFE_TIME)
    return (
        str(round(wp.transform.location.x, 1)),
        str(round(wp.transform.location.y, 1)),
        str(round(wp.transform.location.z, 1)),
        str(round(wp.transform.rotation.yaw, 1))
    )

def get_location_data(a_name, scen_type, tmap, world, spectator, a_type):
    input(f"\033[1m> Enter the '{a_name}' location \033[0m")
    if "sidewalk" in a_type:
        lane_type = carla.LaneType.Sidewalk
    elif "bicycle" in a_type:
        lane_type = carla.LaneType.Biking
    elif "driving" in a_type:
        lane_type = carla.LaneType.Driving
    else:
        lane_type = carla.LaneType.Driving

    wp = tmap.get_waypoint(spectator.get_location(), lane_type=lane_type)
    world.debug.draw_point(wp.transform.location + carla.Location(z=0.2), size=0.1, color=carla.Color(125, 0, 0))
    world.debug.draw_string(wp.transform.location + carla.Location(z=0.5), scen_type, True, color=carla.Color(0, 0 , 125), life_time=LIFE_TIME)

    loc =  (
            str(round(wp.transform.location.x, 1)),
            str(round(wp.transform.location.y, 1)),
            str(round(wp.transform.location.z, 1))
        )

    if "probability" in a_type:
        p = input(f"\033[1m> Enter the '{a_name}' probability \033[0m")
        loc += (p,)
    return loc


def get_value_data(a_name):
    value = input(f"\033[1m> Specify the '{a_name}' value \033[0m")
    return value

def get_interval_data(a_name):
    lower_value = input(f"\033[1m> Specify the '{a_name}' from \033[0m")
    upper_value = input(f"\033[1m> Specify the '{a_name}' from \033[0m{lower_value}\033[1m to \033[0m")
    return (lower_value, upper_value)

def print_scenario_data(scen_type, scen_attributes):
    print_dict = f"\n   scenario_type: {scen_type}\n"
    for print_attribute in scen_attributes:
        print_dict += f"   {print_attribute[0]}: {print_attribute[2]}\n"
    print(print_dict)

def save_scenario(filename, route_id, scenario_type, scenario_attributes):

    while True:
        save = input("\033[1m> Save the scenario? ('Yes' / 'No'): \033[0m")
        if save == "Yes":
            break
        elif save == "No":
            return
        else:
            print(f"\033[1m\033[93mThis is a 'Yes' or 'No' question, try again.\033[0m")

    tree = etree.parse(filename)
    root = tree.getroot()

    scenario_names = {}
    for scen_type in list(SCENARIO_TYPES):
        scenario_names[scen_type] = 1

    found_id = False

    for route in root.iter("route"):
        if route.attrib['id'] != route_id:
            continue

        found_id = True

        scenarios = route.find('scenarios')
        for scenario in scenarios.iter('scenario'):
            scen_type = scenario.attrib['type']
            scenario_names[scen_type] += 1

        number = scenario_names[scenario_type]
        new_scenario = etree.SubElement(scenarios, "scenario")

        new_scenario.set("name", scenario_type + "_" + str(number))
        new_scenario.set("type", scenario_type)

        for a_name, a_type, a_value in scenario_attributes:
            data = etree.SubElement(new_scenario, a_name)
            if a_type == 'transform':
                data.set("x", a_value[0])
                data.set("y", a_value[1])
                data.set("z", a_value[2])
                data.set("yaw", a_value[3])
            elif 'location' in a_type:
                data.set("x", a_value[0])
                data.set("y", a_value[1])
                data.set("z", a_value[2])
                if 'probability' in a_type:
                    data.set("p", a_value[3])
            elif a_type in ('value', 'choice', 'bool'):
                data.set("value", a_value)
            elif a_type == 'interval':
                data.set("from", a_value[0])
                data.set("to", a_value[1])
        break

    if not found_id:
        print(f"\n\033[91mCouldn't find the id '{route_id} in the given routes file\033[0m")
        return

    prettify_and_save_tree(filename, tree)

def prettify_and_save_tree(filename, tree):
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
    new_data = temp.replace(" mie_scattering_scale", "\n" + weather_spaces + "mie_scattering_scale")

    with open(filename, 'w') as f:
        f.write(new_data)

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', required=True, nargs="+", help='File at which to place the scenarios')
    argparser.add_argument('-s', '--show-only', action='store_true', help='Only shows the route')
    args = argparser.parse_args()

    # Get the client
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    # # Get the rest
    world = client.get_world()
    spectator = world.get_spectator()
    tmap = world.get_map()

    file_path = args.file[0]
    route_id = args.file[1] if len(args.file) > 1 else 0

    # Get the data already at the file
    show_saved_scenarios(file_path, route_id, world)
    if args.show_only:
        sys.exit(0)

    print(" ------------------------------------------------------------ ")
    print(" |               Use Ctrl+C to stop the script              | ")
    print(" |            Any ongoing scenario will be ignored          | ")
    print(" |                                                          | ")
    print(" |   Transform and location parameters will automatically   | ")
    print(" |     get the closest waypoint to the spectator camera     | ")
    try:
        while True:
            print(" ------------------------------------------------------------ ")

            # Get the scenario type
            scen_type, trigger_point = get_scenario_type(tmap, world, spectator)

            # Get the attributes
            scen_attributes = get_attributes_data(scen_type, trigger_point, tmap, world, spectator)

            # Give feedback to the user
            print_scenario_data(scen_type, scen_attributes)

            # Save the data
            save_scenario(file_path, route_id, scen_type, scen_attributes)

    except KeyboardInterrupt as e:
        print("\n Detected a keyboard interruption, stopping the script ")

if __name__ == '__main__':
    try:
        main()
    except RuntimeError as e:
        print(e)
