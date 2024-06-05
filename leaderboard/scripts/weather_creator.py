#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import argparse
import carla



def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', action="store_true", help='File to start')
    argparser.add_argument('-r', '--route', default="", help='Route')
    # argparser.add_argument('-t', '--time', default=60, help='Time')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    weather = world.get_weather()

    weather_string = f"         <weather\n"
    weather_string += f"            route_percentage=\"0\"\n"
    weather_string += f"            cloudiness=\"{weather.cloudiness}\" "
    weather_string += f"precipitation=\"{weather.precipitation}\" "
    weather_string += f"precipitation_deposits=\"{weather.precipitation_deposits}\" "
    weather_string += f"wetness=\"{weather.wetness}\"\n"
    weather_string += f"            wind_intensity=\"{weather.wind_intensity}\" "
    weather_string += f"sun_azimuth_angle=\"{weather.sun_azimuth_angle}\" "
    weather_string += f"sun_altitude_angle=\"{weather.sun_altitude_angle}\"\n"
    weather_string += f"            fog_density=\"{weather.fog_density}\" "
    weather_string += f"fog_distance=\"{weather.fog_distance}\" "
    weather_string += f"fog_falloff=\"{round(weather.fog_falloff, 2)}\" "
    weather_string += f"scattering_intensity=\"{weather.scattering_intensity}\"\n"
    weather_string += f"            mie_scattering_scale=\"{round(weather.mie_scattering_scale, 2)}\"/>"

    print(weather_string)

if __name__ == '__main__':
    try:
        main()
    except RuntimeError as e:
        print(e)

