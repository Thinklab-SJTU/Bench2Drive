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

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class RouteLightsBehavior(py_trees.behaviour.Behaviour):

    """
    Behavior responsible for turning the street lights on and off depending on the weather conditions.
    Only those around the ego vehicle will be turned on, regardless of weather conditions
    """
    SUN_ALTITUDE_THRESHOLD_1 = 15
    SUN_ALTITUDE_THRESHOLD_2 = 165

    # For higher fog and cloudness values, the amount of light in scene starts to rapidly decrease
    CLOUDINESS_THRESHOLD = 80
    FOG_THRESHOLD = 40

    # In cases where more than one weather conditition is active, decrease the thresholds
    COMBINED_THRESHOLD = 10

    def __init__(self, ego_vehicle, radius=50, radius_increase=15, name="LightsBehavior"):
        """
        Setup parameters
        """
        super().__init__(name)
        self._ego_vehicle = ego_vehicle
        self._radius = radius
        self._radius_increase = radius_increase
        self._world = CarlaDataProvider.get_world()
        self._light_manager = self._world.get_lightmanager()
        self._light_manager.set_day_night_cycle(False)
        self._vehicle_lights = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam

        self._prev_night_mode = False

    def update(self):
        """
        Turns on / off all the lghts around a radius of the ego vehicle
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self._ego_vehicle)
        if not location:
            return new_status

        night_mode = self._get_night_mode(self._world.get_weather())
        if night_mode:
            self._turn_close_lights_on(location)
        elif self._prev_night_mode:
            self._turn_all_lights_off()

        self._prev_night_mode = night_mode
        return new_status

    def _get_night_mode(self, weather):
        """Check wheather or not the street lights need to be turned on"""
        altitude_dist = weather.sun_altitude_angle - self.SUN_ALTITUDE_THRESHOLD_1
        altitude_dist = min(altitude_dist, self.SUN_ALTITUDE_THRESHOLD_2 - weather.sun_altitude_angle)
        cloudiness_dist = self.CLOUDINESS_THRESHOLD - weather.cloudiness
        fog_density_dist = self.FOG_THRESHOLD - weather.fog_density

        # Check each parameter independetly
        if altitude_dist < 0 or cloudiness_dist < 0 or fog_density_dist < 0:
            return True

        # Check if two or more values are close to their threshold
        joined_threshold = int(altitude_dist < self.COMBINED_THRESHOLD)
        joined_threshold += int(cloudiness_dist < self.COMBINED_THRESHOLD)
        joined_threshold += int(fog_density_dist < self.COMBINED_THRESHOLD)

        if joined_threshold >= 2:
            return True

        return False

    def _turn_close_lights_on(self, location):
        """Turns on the lights of all the objects close to the ego vehicle"""
        ego_speed = CarlaDataProvider.get_velocity(self._ego_vehicle)
        radius = max(self._radius, self._radius_increase * ego_speed)

        # Street lights
        on_lights = []
        off_lights = []

        all_lights = self._light_manager.get_all_lights()
        for light in all_lights:
            if light.location.distance(location) > radius:
                if light.is_on:
                    off_lights.append(light)
            else:
                if not light.is_on:
                    on_lights.append(light)

        self._light_manager.turn_on(on_lights)
        self._light_manager.turn_off(off_lights)

        # Vehicles
        all_vehicles = CarlaDataProvider.get_all_actors().filter('*vehicle.*')
        scenario_vehicles = [v for v in all_vehicles if v.attributes['role_name'] == 'scenario']

        for vehicle in scenario_vehicles:
            try:
                if vehicle.get_location().distance(location) > radius:
                        lights = vehicle.get_light_state()
                        lights &= ~self._vehicle_lights  # Remove those lights
                        vehicle.set_light_state(carla.VehicleLightState(lights))
                else:
                    lights = vehicle.get_light_state()
                    lights |= self._vehicle_lights  # Add those lights
                    vehicle.set_light_state(carla.VehicleLightState(lights))
            except RuntimeError:
                pass

        # Ego vehicle
        lights = self._ego_vehicle.get_light_state()
        lights |= self._vehicle_lights
        self._ego_vehicle.set_light_state(carla.VehicleLightState(lights))

    def _turn_all_lights_off(self):
        """Turns off the lights of all object"""
        all_lights = self._light_manager.get_all_lights()
        off_lights = [l for l in all_lights if l.is_on]
        self._light_manager.turn_off(off_lights)

        # Vehicles
        all_vehicles = CarlaDataProvider.get_all_actors().filter('*vehicle.*')
        scenario_vehicles = [v for v in all_vehicles if v.attributes['role_name'] == 'scenario']

        for vehicle in scenario_vehicles:
            lights = vehicle.get_light_state()
            lights &= ~self._vehicle_lights  # Remove those lights
            vehicle.set_light_state(carla.VehicleLightState(lights))

        # Ego vehicle
        lights = self._ego_vehicle.get_light_state()
        lights &= ~self._vehicle_lights  # Remove those lights
        self._ego_vehicle.set_light_state(carla.VehicleLightState(lights))

    def terminate(self, new_status):
        self._light_manager.set_day_night_cycle(True)
        return super().terminate(new_status)