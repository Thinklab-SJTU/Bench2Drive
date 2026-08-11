import json
import carla
import argparse
import xml.etree.ElementTree as ET
from agents.navigation.global_route_planner import GlobalRoutePlanner
import os
import atexit
import subprocess
import time
import random

Ability = {
    "Overtaking":['Accident', 'AccidentTwoWays', 'ConstructionObstacle', 'ConstructionObstacleTwoWays', 'HazardAtSideLaneTwoWays', 'HazardAtSideLane', 'ParkedObstacleTwoWays', 'ParkedObstacle', 'VehicleOpensDoorTwoWays'],
    "Merging": ['CrossingBicycleFlow', 'EnterActorFlow', 'HighwayExit', 'InterurbanActorFlow', 'HighwayCutIn', 'InterurbanAdvancedActorFlow', 'MergerIntoSlowTrafficV2', 'MergerIntoSlowTraffic', 'NonSignalizedJunctionLeftTurn', 'NonSignalizedJunctionRightTurn', 'NonSignalizedJunctionLeftTurnEnterFlow', 'ParkingExit', 'SequentialLaneChange', 'SignalizedJunctionLeftTurn', 'SignalizedJunctionRightTurn', 'SignalizedJunctionLeftTurnEnterFlow'],
    "Emergency_Brake": ['BlockedIntersection', 'DynamicObjectCrossing', 'HardBreakRoute', 'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight', 'ParkingCutIn', 'PedestrianCrossing', 'ParkingCrossingPedestrian', 'StaticCutIn', 'VehicleTurningRoute', 'VehicleTurningRoutePedestrian', 'ControlLoss'],
    "Give_Way": ['InvadingTurn', 'YieldToEmergencyVehicle'],
    "Traffic_Signs": ['BlockedIntersection', 'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight', 'PedestrianCrossing', 'VehicleTurningRoute', 'VehicleTurningRoutePedestrian', 'EnterActorFlow', 'CrossingBicycleFlow', 'NonSignalizedJunctionLeftTurn', 'NonSignalizedJunctionRightTurn', 'NonSignalizedJunctionLeftTurnEnterFlow', 'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight', 'PedestrianCrossing', 'SignalizedJunctionLeftTurn', 'SignalizedJunctionRightTurn', 'SignalizedJunctionLeftTurnEnterFlow', 'T_Junction', 'VanillaNonSignalizedTurn', 'VanillaSignalizedTurnEncounterGreenLight', 'VanillaSignalizedTurnEncounterRedLight', 'VanillaNonSignalizedTurnEncounterStopsign', 'VehicleTurningRoute', 'VehicleTurningRoutePedestrian']
}

def get_infraction_status(record):
    for infraction,  value in record['infractions'].items():
        if infraction == "min_speed_infractions":
            continue
        elif len(value) > 0:
            return True
    return False

def update_Ability(scenario_name, Ability_Statistic, status):
    for ability, scenarios in Ability.items():
        if scenario_name in scenarios:
            Ability_Statistic[ability][1] += 1
            if status:
                Ability_Statistic[ability][0] += 1
    pass

def update_Success(scenario_name, Success_Statistic, status):
    if scenario_name not in Success_Statistic:
        if status:
            Success_Statistic[scenario_name] = [1, 1]
        else:
            Success_Statistic[scenario_name] = [0, 1]
    else:
        Success_Statistic[scenario_name][1] += 1
        if status:
            Success_Statistic[scenario_name][0] += 1
    pass

def get_position(xml_route):
    waypoints_elem = xml_route.find('waypoints')
    keypoints = waypoints_elem.findall('position')
    return [carla.Location(float(pos.get('x')), float(pos.get('y')), float(pos.get('z'))) for pos in keypoints]

def get_route_result(records, route_id):
    for record in records:
        record_route_id = record['route_id'].split('_')[1]
        if route_id == record_route_id:
            return record
    return None

def get_waypoint_route(locs, grp):
    route = []
    for i in range(len(locs) - 1):
        loc = locs[i]
        loc_next = locs[i + 1]
        interpolated_trace = grp.trace_route(loc, loc_next)
        for wp, _ in interpolated_trace:
            route.append(wp)
    return route

# ===== OFFLINE MAP LOADER =====
def load_offline_map(town_name, map_root):
    xodr1 = os.path.join(map_root, 'OpenDrive', f'{town_name}.xodr')
    xodr2 = os.path.join(map_root, town_name, 'OpenDrive', f'{town_name}.xodr')

    if os.path.exists(xodr1):
        with open(xodr1, 'r') as f:
            xodr = f.read()
    elif os.path.exists(xodr2):
        with open(xodr2, 'r') as f:
            xodr = f.read()
    else:
        raise FileNotFoundError(f'OpenDrive file not found for {town_name}')

    return carla.Map(town_name, xodr)


def main(args):
    routes_file = args.file
    result_file = args.result_file
    map_root = os.path.join(os.environ.get("CARLA_ROOT", "."), "CarlaUE4/Content/Carla/Maps")   # OFFLINE MOD
    print(f"[debug] map_root = {map_root}")

    Ability_Statistic = {k: [0, 0] for k in Ability}  # [success_count, total_count]
    Success_Statistic = {}
    crash_route_list = []

    with open(result_file, 'r') as f:
        data = json.load(f)
    records = data["_checkpoint"]["records"]

    tree = ET.parse(routes_file)
    root = tree.getroot()
    routes = sorted(root.findall('route'), key=lambda x: x.get('town'))

    current_town = None
    carla_map = None
    grp = None

    for route in routes:
        scenario_name = route.find('scenarios').find('scenario').get("type")
        route_id = route.get('id')
        town_name = route.get('town')

        route_record = get_route_result(records, route_id)
        if route_record is None:
            crash_route_list.append((scenario_name, route_id))
            continue

        if route_record["status"] in ['Completed', 'Perfect'] and \
           not get_infraction_status(route_record):
            record_success_status = True
        else:
            record_success_status = False

        update_Ability(scenario_name, Ability_Statistic, record_success_status)
        update_Success(scenario_name, Success_Statistic, record_success_status)

        if scenario_name in Ability["Traffic_Signs"] and not record_success_status:
            if town_name != current_town:
                current_town = town_name
                carla_map = load_offline_map(current_town, map_root)
                grp = GlobalRoutePlanner(carla_map, 1.0)

            location_list = get_position(route)
            waypoint_route = get_waypoint_route(location_list, grp)

            count = 0
            for wp in waypoint_route:
                count += 1
                if wp.is_junction:
                    break
            if not wp.is_junction:
                raise RuntimeError("No junction found in route")

            junction_completion = float(count + 8) / len(waypoint_route)
            record_completion = route_record["scores"]["score_route"] / 100.0

            stop_infraction = route_record["infractions"]["stop_infraction"]
            red_light_infraction = route_record["infractions"]["red_light"]

            if record_completion > junction_completion and \
               not stop_infraction and not red_light_infraction:
                Ability_Statistic['Traffic_Signs'][0] += 1

    Ability_Res = {k: v[0] / v[1] for k, v in Ability_Statistic.items()}
    Ability_Res['mean'] = sum(Ability_Res.values()) / len(Ability_Res)
    Ability_Res['crashed'] = crash_route_list

    print(f"Result saved to {'.'.join(result_file.split('.')[:-1])}_ability.json")
    with open(f"{'.'.join(result_file.split('.')[:-1])}_ability.json", 'w') as f:
        json.dump(Ability_Res, f, indent=4)

    print("Mean:", Ability_Res['mean'])
    print("Crashed Route num:", len(crash_route_list))
    print("Finished!")


if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-f', '--file', default="leaderboard/data/bench2drive220.xml")
    argparser.add_argument('-r', '--result_file', required=True)
    args = argparser.parse_args()
    main(args)