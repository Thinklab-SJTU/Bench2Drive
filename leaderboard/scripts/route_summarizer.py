#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import os
import sys
from lxml import etree
from tabulate import tabulate

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

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

def convert_elem_to_location(elem):
    """Convert an ElementTree.Element to a CARLA Location"""
    return carla.Location(float(elem.attrib.get('x')), float(elem.attrib.get('y')), float(elem.attrib.get('z')))

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', required=True, help="Route's file path")
    argparser.add_argument('--endpoint', default="", help='Output file')
    argparser.add_argument('--show', action="store_true", help='Print the results on stdout')
    args = argparser.parse_args()

    if not args.endpoint and not args.show:
        print("No output method was selected. Use either '--endpoint', or '--show' to get the route results")
        sys.exit(0)

    root = etree.parse(args.file).getroot()
    total_distance = 0
    total_num_scenarios = 0
    total_scenarios = {}

    statistics = [['Route id', 'Distance (m)', 'Scenarios (type)', 'Scenarios (nº)', 'Avg dist between scenarios']]

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
            grp = GlobalRoutePlanner(tmap, 2.0)

        print(f"Parsing route '{route_id}'")

        # Get the route distance
        route_distance = 0

        route_points = []
        for position in route.find('waypoints').iter('position'):
            route_points.append(convert_elem_to_location(position))

        for i in range(len(route_points) - 1):
            waypoint = route_points[i]
            waypoint_next = route_points[i + 1]
            interpolated_trace = grp.trace_route(waypoint, waypoint_next)
            for j in range(len(interpolated_trace) - 1):
                wp = interpolated_trace[j][0]
                wp_next = interpolated_trace[j + 1][0]
                route_distance += wp.transform.location.distance(wp_next.transform.location)

        # Get the route scenarios
        route_scenarios = {}

        for scenario in route.find('scenarios').iter('scenario'):
            scenario_type = scenario.attrib.get('type')
            if not scenario_type in route_scenarios:
                route_scenarios[scenario_type] = 0

            route_scenarios[scenario_type] += 1

        num_scenarios = sum([x for x in route_scenarios.values()])
        dist_scenarios = route_distance / num_scenarios

        print_scenarios = ""
        route_scenario_items = sorted(list(route_scenarios.items()), key=lambda x: x[0])
        for s_type, s_num in route_scenario_items:
            print_scenarios += f"{s_type}: {s_num}\n"

        statistics += [[
            route_id,
            round(route_distance, 1),
            print_scenarios,
            sum([x for x in route_scenarios.values()]),
            round(dist_scenarios, 1)
        ]]

        # Get the total amounts
        total_distance += route_distance
        total_num_scenarios += num_scenarios

        for s_type, s_num in route_scenario_items:
            if s_type not in total_scenarios:
                total_scenarios[s_type] = 0
            total_scenarios[s_type] += s_num

        prev_town = route_town

    print_total_scenarios = ""
    total_scenario_items = sorted(list(total_scenarios.items()), key=lambda x: x[0])
    for s_type, s_num in total_scenario_items:
        print_total_scenarios += f"{s_type}: {s_num}\n"

    statistics += [[
        "Total",
        round(total_distance, 1),
        print_total_scenarios,
        total_num_scenarios,
        round(total_distance / total_num_scenarios, 1)
    ]]

    output = tabulate(statistics, tablefmt='fancy_grid')
    if args.show:
        print(output)

    if args.endpoint:
        with open(args.endpoint, 'w', encoding='utf-8') as fd:
            fd.write(output)

if __name__ == '__main__':
    try:
        main()
    except RuntimeError as e:
        print(e)
