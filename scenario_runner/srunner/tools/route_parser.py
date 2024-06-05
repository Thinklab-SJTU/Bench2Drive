#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Module used to parse all the route and scenario configuration parameters.
"""

import json
import math
import xml.etree.ElementTree as ET

import carla
from agents.navigation.local_planner import RoadOption
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData

# Threshold to say if a scenarios trigger position is part of the route
DIST_THRESHOLD = 2.0
ANGLE_THRESHOLD = 10


def convert_elem_to_transform(elem):
    """Convert an ElementTree.Element to a CARLA transform"""
    return carla.Transform(
        carla.Location(
            float(elem.attrib.get('x')),
            float(elem.attrib.get('y')),
            float(elem.attrib.get('z'))
        ),
        carla.Rotation(
            roll=0.0,
            pitch=0.0,
            yaw=float(elem.attrib.get('yaw'))
        )
    )


class RouteParser(object):

    """
    Pure static class used to parse all the route and scenario configuration parameters.
    """

    @staticmethod
    def parse_routes_file(route_filename, single_route_id=''):
        """
        Returns a list of route configuration elements.
        :param route_filename: the path to a set of routes.
        :param single_route: If set, only this route shall be returned
        :return: List of dicts containing the waypoints, id and town of the routes
        """

        route_configs = []
        tree = ET.parse(route_filename)
        for route in tree.iter("route"):

            route_id = route.attrib['id']
            if single_route_id and route_id != single_route_id:
                continue

            route_config = RouteScenarioConfiguration()
            route_config.town = route.attrib['town']
            route_config.name = "RouteScenario_{}".format(route_id)
            route_config.weather = RouteParser.parse_weather(route)

            # The list of carla.Location that serve as keypoints on this route
            positions = []
            for position in route.find('waypoints').iter('position'):
                positions.append(carla.Location(x=float(position.attrib['x']),
                                                y=float(position.attrib['y']),
                                                z=float(position.attrib['z'])))
            route_config.keypoints = positions

            # The list of ScenarioConfigurations that store the scenario's data
            scenario_configs = []
            for scenario in route.find('scenarios').iter('scenario'):
                scenario_config = ScenarioConfiguration()
                scenario_config.name = scenario.attrib.get('name')
                scenario_config.type = scenario.attrib.get('type')

                for elem in scenario.getchildren():
                    if elem.tag == 'trigger_point':
                        scenario_config.trigger_points.append(convert_elem_to_transform(elem))
                    elif elem.tag == 'other_actor':
                        scenario_config.other_actors.append(ActorConfigurationData.parse_from_node(elem, 'scenario'))
                    else:
                        scenario_config.other_parameters[elem.tag] = elem.attrib

                scenario_configs.append(scenario_config)
            route_config.scenario_configs = scenario_configs

            route_configs.append(route_config)

        return route_configs

    @staticmethod
    def parse_weather(route):
        """
        Parses all the weather information as a list of [position, carla.WeatherParameters],
        where the position represents a % of the route.
        """
        weathers = []

        weathers_elem = route.find("weathers")
        if weathers_elem is None:
            return [[0, carla.WeatherParameters(sun_altitude_angle=70, cloudiness=50)]]

        for weather_elem in weathers_elem.iter('weather'):
            route_percentage = float(weather_elem.attrib['route_percentage'])

            weather = carla.WeatherParameters(sun_altitude_angle=70, cloudiness=50)  # Base weather
            for weather_attrib in weather_elem.attrib:
                if hasattr(weather, weather_attrib):
                    setattr(weather, weather_attrib, float(weather_elem.attrib[weather_attrib]))
                elif weather_attrib != 'route_percentage':
                    print(f"WARNING: Ignoring '{weather_attrib}', as it isn't a weather parameter")

            weathers.append([route_percentage, weather])

        weathers.sort(key=lambda x: x[0])
        return weathers

    @staticmethod
    def is_scenario_at_route(trigger_transform, route):
        """
        Check if the scenario is affecting the route.
        This is true if the trigger position is very close to any route point
        """
        def is_trigger_close(trigger_transform, route_transform):
            """Check if the two transforms are similar"""
            dist = trigger_transform.location.distance(route_transform.location)
            angle_dist = (trigger_transform.rotation.yaw - route_transform.rotation.yaw) % 360

            return dist < DIST_THRESHOLD \
                and (angle_dist < ANGLE_THRESHOLD or angle_dist > (360 - ANGLE_THRESHOLD))

        for route_transform, _ in route:
            if is_trigger_close(trigger_transform, route_transform):
                return True

        return False
