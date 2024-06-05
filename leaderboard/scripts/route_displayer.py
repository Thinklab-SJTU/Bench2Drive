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
from agents.navigation.local_planner import RoadOption
from numpy import random

LIFE_TIME = 10000

COLORS = [
    carla.Color(255, 255, 255),
    carla.Color(255, 0, 0),
    carla.Color(0, 255, 0),
    carla.Color(0, 0, 255),
    carla.Color(255, 255, 0),
    carla.Color(255, 0, 255),
    carla.Color(0, 255, 255),

    carla.Color(192, 192, 192),
    carla.Color(192, 0, 0),
    carla.Color(0, 192, 0),
    carla.Color(0, 0, 192),
    carla.Color(192, 192, 0),
    carla.Color(192, 0, 192),
    carla.Color(0, 192, 192),

    carla.Color(128, 128, 128),
    carla.Color(128, 0, 0),
    carla.Color(0, 128, 0),
    carla.Color(0, 0, 128),
    carla.Color(128, 128, 0),
    carla.Color(128, 0, 128),
    carla.Color(0, 128, 128),

    carla.Color(64, 64, 64),
    carla.Color(64, 0, 0),
    carla.Color(0, 64, 0),
    carla.Color(0, 0, 64),
    carla.Color(64, 64, 0),
    carla.Color(64, 0, 64),
    carla.Color(0, 64, 64),

    carla.Color(0, 0, 0),
]


def draw_point(world, wp, color):
    world.debug.draw_point( wp.transform.location + carla.Location(z=0.2), size=0.05, color=color, life_time=LIFE_TIME)


def draw_keypoint(world, location):
    world.debug.draw_point(location + carla.Location(z=0.2), size=0.1, color=carla.Color(128, 0, 128), life_time=LIFE_TIME)
    string = "(" + str(round(location.x, 1)) + ", " + str(round(location.y, 1)) + ", " + str(round(location.z, 1)) + ")"
    world.debug.draw_string(location + carla.Location(z=0.5), string, True, color=carla.Color(0, 0 , 128), life_time=LIFE_TIME)


def show_all_routes(filename, show_scenarios, world, grp):
    def convert_elem_to_location(elem):
        """Convert an ElementTree.Element to a CARLA Location"""
        return carla.Location(float(elem.attrib.get('x')), float(elem.attrib.get('y')), float(elem.attrib.get('z')))

    def get_random_color(rng):
        """Gets a random CARLA RGB color"""
        r = rng.randint(0, 256)
        g = rng.randint(0, 256)
        b = rng.randint(0, 256)
        return carla.Color(r, g, b)

    rng = random.RandomState(2002)
    colors = list(COLORS)
    rng.shuffle(colors)

    tree = etree.parse(filename)
    root = tree.getroot()
    for i, route in enumerate(root.iter("route")):
        prev_point = None
        color = get_random_color(rng)
        # color = colors[i]

        for position in route.find('waypoints').iter('position'):
            point = convert_elem_to_location(position)
            # draw_keypoint(world, point)

            if prev_point:
                interpolated_trace = grp.trace_route(prev_point, point)
                for wp, _ in interpolated_trace:
                    draw_point(world, wp, color)
            else:
                world.debug.draw_string(point + carla.Location(z=1), route.attrib['id'], color=color, life_time=10000)
            prev_point = point

        if show_scenarios:
            show_saved_scenarios(route, world)


def show_route(filename, route_id, show_keypoints, show_scenarios, world, grp):
    def convert_elem_to_location(elem):
        """Convert an ElementTree.Element to a CARLA Location"""
        return carla.Location(float(elem.attrib.get('x')), float(elem.attrib.get('y')), float(elem.attrib.get('z')))
    def get_color(option):
        if option == RoadOption.LEFT:  # Yellow
            return carla.Color(128, 128, 0)
        elif option == RoadOption.RIGHT:  # Cyan
            return carla.Color(0, 128, 128)
        elif option == RoadOption.CHANGELANELEFT:  # Orange
            return carla.Color(128, 32, 0)
        elif option == RoadOption.CHANGELANERIGHT:  # Dark Cyan
            return carla.Color(0, 32, 128)
        elif option == RoadOption.STRAIGHT:  # Gray
            return carla.Color(64, 64, 64)
        else:  # LANEFOLLOW
            return carla.Color(0, 128, 0)  # Green

    tree = etree.parse(filename)
    root = tree.getroot()

    for route in root.iter("route"):
        if route.attrib['id'] != route_id:
            continue

        points = []
        for position in route.find('waypoints').iter('position'):
            points.append(convert_elem_to_location(position))

        if points:
            for i in range(len(points) - 1):
                waypoint = points[i]
                waypoint_next = points[i + 1]
                interpolated_trace = grp.trace_route(waypoint, waypoint_next)
                for j in range(len(interpolated_trace) - 1):
                    wp, option = interpolated_trace[j]
                    draw_point(world, wp, get_color(option))

                if show_keypoints:
                    draw_keypoint(world, waypoint)
            if show_keypoints:
                draw_keypoint(world, points[-1])

        if show_scenarios:
            show_saved_scenarios(route, world)

        # spec_transform = carla.Transform(points[0] + carla.Location(z=100), carla.Rotation(pitch=-90))
        # world.get_spectator().set_transform(spec_transform)

def show_saved_scenarios(route, world):
    def convert_elem_to_location(elem):
        """Convert an ElementTree.Element to a CARLA Location"""
        return carla.Location(float(elem.attrib.get('x')), float(elem.attrib.get('y')), float(elem.attrib.get('z')))

    for scenario in route.find('scenarios').iter('scenario'):
        name = scenario.attrib.get('name')
        trigger_location = convert_elem_to_location(scenario.find('trigger_point'))
        world.debug.draw_point(trigger_location + carla.Location(z=0.2), size=0.1, color=carla.Color(125, 0, 0))
        world.debug.draw_string(trigger_location + carla.Location(z=0.5), name, True, color=carla.Color(0, 0 , 125), life_time=LIFE_TIME)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', required=True, help='File at which to place the scenarios')
    argparser.add_argument('-sr', '--show-route', help='Shows the given route')
    argparser.add_argument('-sa', '--show-all', action='store_true', help='Shows all the routes')
    argparser.add_argument('-sk', '--show-keypoints', action='store_true', help='Shows the keypoints that define the route')
    argparser.add_argument('-ss', '--show-scenarios', action='store_true', help='Shows the scemarios part of the route')
    args = argparser.parse_args()

    # Get the client
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    # Get the rest
    world = client.get_world()
    tmap = world.get_map()
    grp = GlobalRoutePlanner(tmap, 2.0)

    if not args.show_route and not args.show_all:
        print("No instructions to show were given, stopping the script")

    # Show all routes
    if args.show_all:
        show_all_routes(args.file, args.show_scenarios, world, grp)

    # Show one route
    if args.show_route:
        show_route(args.file, args.show_route, args.show_keypoints, args.show_scenarios, world, grp)


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as e:
        print(e)
