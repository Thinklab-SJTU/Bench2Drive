#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a weather class and py_trees behavior
to simulate weather in CARLA according to the astronomic
behavior of the sun.
"""

import datetime
import math
import operator

import ephem
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime


class Weather(object):

    """
    Class to simulate weather in CARLA according to the astronomic behavior of the sun

    The sun position (azimuth and altitude angles) is obtained by calculating its
    astronomic position for the CARLA reference position (x=0, y=0, z=0) using the ephem
    library.

    Args:
        carla_weather (carla.WeatherParameters): Initial weather settings.
        dtime (datetime): Initial date and time in UTC (required for animation only).
            Defaults to None.
        animation (bool): Flag to allow animating the sun position over time.
            Defaults to False.

    Attributes:
        carla_weather (carla.WeatherParameters): Weather parameters for CARLA.
        animation (bool): Flag to allow animating the sun position over time.
        _sun (ephem.Sun): The sun as astronomic entity.
        _observer_location (ephem.Observer): Holds the geographical position (lat/lon/altitude)
            for which the sun position is obtained.
        datetime (datetime): Date and time in UTC (required for animation only).
    """

    def __init__(self, carla_weather, dtime=None, animation=False):
        """
        Class constructor
        """
        self.carla_weather = carla_weather
        self.animation = animation

        self._sun = ephem.Sun()  # pylint: disable=no-member
        self._observer_location = ephem.Observer()
        geo_location = CarlaDataProvider.get_map().transform_to_geolocation(carla.Location(0, 0, 0))
        self._observer_location.lon = str(geo_location.longitude)
        self._observer_location.lat = str(geo_location.latitude)

        # @TODO This requires the time to be in UTC to be accurate
        self.datetime = dtime
        if self.datetime:
            self._observer_location.date = self.datetime

        self.update()

    def update(self, delta_time=0):
        """
        If the weather animation is true, the new sun position is calculated w.r.t delta_time

        Nothing happens if animation or datetime are None.

        Args:
            delta_time (float): Time passed since self.datetime [seconds].
        """
        if not self.animation or not self.datetime:
            return

        self.datetime = self.datetime + datetime.timedelta(seconds=delta_time)
        self._observer_location.date = self.datetime

        self._sun.compute(self._observer_location)
        self.carla_weather.sun_altitude_angle = math.degrees(self._sun.alt)
        self.carla_weather.sun_azimuth_angle = math.degrees(self._sun.az)


class OSCWeatherBehavior(py_trees.behaviour.Behaviour):

    """
    Atomic to read weather settings from the blackboard and apply these in CARLA.
    Used in combination with UpdateWeather() to have a continuous weather simulation.

    This behavior is always in a running state and must never terminate.
    The user must not add this behavior. It is automatically added by the ScenarioManager.

    This atomic also sets the datetime to blackboard variable, used by TimeOfDayComparison atomic

    Args:
        name (string): Name of the behavior.
            Defaults to 'WeatherBehavior'.

    Attributes:
        _weather (srunner.scenariomanager.weather_sim.Weather): Weather settings.
        _current_time (float): Current CARLA time [seconds].
    """

    def __init__(self, name="WeatherBehavior"):
        """
        Setup parameters
        """
        super(OSCWeatherBehavior, self).__init__(name)
        self._weather = None
        self._current_time = None

    def initialise(self):
        """
        Set current time to current CARLA time
        """
        self._current_time = GameTime.get_time()

    def update(self):
        """
        Check if new weather settings are available on the blackboard, and if yes fetch these
        into the _weather attribute.

        Apply the weather settings from _weather to CARLA.

        Note:
            To minimize CARLA server interactions, the weather is only updated, when the blackboard
            is updated, or if the weather animation flag is true. In the latter case, the update
            frequency is 1 Hz.

        returns:
            py_trees.common.Status.RUNNING
        """

        weather = None

        try:
            check_weather = operator.attrgetter("CarlaWeather")
            weather = check_weather(py_trees.blackboard.Blackboard())
        except AttributeError:
            pass

        if weather:
            self._weather = weather
            delattr(py_trees.blackboard.Blackboard(), "CarlaWeather")
            CarlaDataProvider.get_world().set_weather(self._weather.carla_weather)
            py_trees.blackboard.Blackboard().set("Datetime", self._weather.datetime, overwrite=True)

        if self._weather and self._weather.animation:
            new_time = GameTime.get_time()
            delta_time = new_time - self._current_time

            if delta_time > 1:
                self._weather.update(delta_time)
                self._current_time = new_time
                CarlaDataProvider.get_world().set_weather(self._weather.carla_weather)

                py_trees.blackboard.Blackboard().set("Datetime", self._weather.datetime, overwrite=True)

        return py_trees.common.Status.RUNNING


class RouteWeatherBehavior(py_trees.behaviour.Behaviour):

    """
    Given a set of route weathers ([position, carla.WeatherParameters]),
    monitors the ego vehicle to dynamically change the weather as the ego advanced through the route.

    This behavior interpolates the desired weather between two weather keypoints and if the extreme cases
    (0% and 100%) aren't defined, the closest one will be chosen
    (i.e, if the route weather is at 90%, all weathers from 90% to 100% will be the one defined at 90%)

    Use the debug argument to print what is the route's percentage of each route position.
    """

    def __init__(self, ego_vehicle, route, weathers, debug=False, name="RouteWeatherBehavior"):
        """
        Setup parameters
        """
        super().__init__(name)
        self._world = CarlaDataProvider.get_world()
        self._ego_vehicle = ego_vehicle
        self._route = route

        self._weathers = weathers
        if self._weathers[0][0] != 0:  # Make sure the weather is defined at 0%
            self._weathers.insert(0, [0, self._weathers[0]])
        if self._weathers[-1][0] != 100:  # Make sure the weather is defined at 100%
            self._weathers.append([100, self._weathers[-1]])

        self._wsize = 3

        self._current_index = 0
        self._route_length = len(self._route)
        self._route_transforms, _ = zip(*self._route)
        self._route_perc = self._get_route_percentages()
        if debug:
            debug_perc = -1
            for transform, perc in zip(self._route_transforms, self._route_perc):
                location = transform.location
                new_perc = int(perc)
                if new_perc > debug_perc:
                    self._world.debug.draw_string(
                        location + carla.Location(z=1),
                        str(new_perc),
                        color=carla.Color(50, 50, 50),
                        life_time=100000
                    )
                    debug_perc = new_perc
        self._route_weathers = self.get_route_weathers()

    def _get_route_percentages(self):
        """
        Calculate the accumulated distance percentage at each point in the route
        """
        accum_m = []
        prev_loc = self._route_transforms[0].location
        for i, tran in enumerate(self._route_transforms):
            new_dist = tran.location.distance(prev_loc)
            added_dist = 0 if i == 0 else accum_m[i - 1]
            accum_m.append(new_dist + added_dist)
            prev_loc = tran.location

        max_dist = accum_m[-1]
        return [x / max_dist * 100 for x in accum_m]

    def get_route_weathers(self):
        """Calculate the desired weather at each point in the route"""
        def interpolate(prev_w, next_w, perc, name):
            x0 = prev_w[0]
            x1 = next_w[0]
            if x0 == x1:
                raise ValueError("Two weather keypoints have the same route percentage")
            y0 = getattr(prev_w[1], name)
            y1 = getattr(next_w[1], name)
            return y0 + (y1 - y0) * (perc - x0) / (x1 - x0)

        route_weathers = []

        weather_index = 0
        prev_w = self._weathers[weather_index]
        next_w = self._weathers[weather_index + 1]

        for perc in self._route_perc:
            if perc > next_w[0]:  # Must be strictly less, or an IndexError will occur at 100%
                weather_index += 1
                prev_w = self._weathers[weather_index]
                next_w = self._weathers[weather_index + 1]

            weather = carla.WeatherParameters()
            weather.cloudiness = interpolate(prev_w, next_w, perc, 'cloudiness')
            weather.precipitation = interpolate(prev_w, next_w, perc, 'precipitation')
            weather.precipitation_deposits = interpolate(prev_w, next_w, perc, 'precipitation_deposits')
            weather.wind_intensity = interpolate(prev_w, next_w, perc, 'wind_intensity')
            weather.sun_azimuth_angle = interpolate(prev_w, next_w, perc, 'sun_azimuth_angle')
            weather.sun_altitude_angle = interpolate(prev_w, next_w, perc, 'sun_altitude_angle')
            weather.wetness = interpolate(prev_w, next_w, perc, 'wetness')
            weather.fog_distance = interpolate(prev_w, next_w, perc, 'fog_distance')
            weather.fog_density = interpolate(prev_w, next_w, perc, 'fog_density')
            weather.fog_falloff = interpolate(prev_w, next_w, perc, 'fog_falloff')
            weather.scattering_intensity = interpolate(prev_w, next_w, perc, 'scattering_intensity')
            weather.mie_scattering_scale = interpolate(prev_w, next_w, perc, 'mie_scattering_scale')
            weather.rayleigh_scattering_scale = interpolate(prev_w, next_w, perc, 'rayleigh_scattering_scale')

            route_weathers.append(weather)

        return route_weathers

    def update(self):
        """
        Check the location of the ego vehicle, updating the weather if it has advanced through the route
        """
        new_status = py_trees.common.Status.RUNNING

        ego_location = CarlaDataProvider.get_location(self._ego_vehicle)
        if ego_location is None:
            return new_status

        new_index = self._current_index

        for index in range(self._current_index, min(self._current_index + self._wsize + 1, self._route_length)):
            route_transform = self._route_transforms[index]
            route_veh_vec = ego_location - route_transform.location
            if route_veh_vec.dot(route_transform.get_forward_vector()) > 0:
                new_index = index

        if new_index > self._current_index:
            self._world.set_weather(self._route_weathers[new_index])
        self._current_index = new_index

        return new_status
