import carla
import numpy as np

import cv2

from pathlib import Path
import os
import argparse
import time
import subprocess
import json

def check_waypoints_status(waypoints_list):
    first_wp = waypoints_list[0]
    init_status = first_wp.is_junction
    current_status = first_wp.is_junction
    change_status_time = 0
    for wp in waypoints_list[1:]:
        if wp.is_junction != current_status:
            current_status = wp.is_junction
            change_status_time += 1
        pass
    if change_status_time == 0:
        return 'Junction' if init_status else 'Normal'
    elif change_status_time == 1:
        return 'EnterNormal' if init_status else 'EnterJunction'
    elif change_status_time == 2:
        return 'PassNormal' if init_status else 'PassJunction'
    else:
        return 'StartJunctionMultiChange' if init_status else 'StartNormalMultiChange'

class TriggerVolumeGettor(object):
    
    @staticmethod
    def get_global_bbx(actor, bbx):
        if actor.is_alive:
            bbx.location = actor.get_transform().transform(bbx.location)
            bbx.rotation = actor.get_transform().rotation
            return bbx
        return None

    @staticmethod
    def get_corners_from_actor_list(actor_list):
        for actor_transform, bb_loc, bb_ext in actor_list:

            corners = [carla.Location(x=-bb_ext.x, y=-bb_ext.y),
                       carla.Location(x=bb_ext.x, y=-bb_ext.y),
                       carla.Location(x=bb_ext.x, y=0),
                       carla.Location(x=bb_ext.x, y=bb_ext.y),
                       carla.Location(x=-bb_ext.x, y=bb_ext.y)]
            corners = [bb_loc + corner for corner in corners]

            corners = [actor_transform.transform(corner) for corner in corners]
            corners = [[corner.x, corner.y, corner.z] for corner in corners]
        return corners
    
    @staticmethod
    def insert_point_into_dict(lane_marking_dict, corners, road_id, parent_actor_location, Volume_Type=None):
        if road_id not in lane_marking_dict.keys():
            print("Cannot find road:", road_id)
            raise
        if Volume_Type is None:
            print("Missing 'Volume Type' ")
            raise
        if 'Trigger_Volumes' not in lane_marking_dict[road_id]:
            lane_marking_dict[road_id]['Trigger_Volumes'] = [{'Points': corners[:], 'Type': Volume_Type, 'ParentActor_Location': parent_actor_location[:]}]
        else:
            lane_marking_dict[road_id]['Trigger_Volumes'].append({'Points': corners[:], 'Type': Volume_Type, 'ParentActor_Location': parent_actor_location[:]})
    
    @staticmethod
    def get_stop_sign_trigger_volume(all_stop_sign_actors, lane_marking_dict, carla_map):
        for actor in all_stop_sign_actors:
            bb_loc = carla.Location(actor.trigger_volume.location)
            bb_ext = carla.Vector3D(actor.trigger_volume.extent)
            bb_ext.x = max(bb_ext.x, bb_ext.y)
            bb_ext.y = max(bb_ext.x, bb_ext.y)
            base_transform = actor.get_transform()
            stop_info_list = [(carla.Transform(base_transform.location, base_transform.rotation), bb_loc, bb_ext)]
            corners = TriggerVolumeGettor.get_corners_from_actor_list(stop_info_list)
            
            trigger_volume_wp = carla_map.get_waypoint(base_transform.transform(bb_loc))
            actor_loc = actor.get_location()
            actor_loc_points = [actor_loc.x, actor_loc.y, actor_loc.z]
            TriggerVolumeGettor.insert_point_into_dict(lane_marking_dict, corners, trigger_volume_wp.road_id, actor_loc_points, Volume_Type='StopSign')
            
        pass
    
    @staticmethod
    def get_traffic_light_trigger_volume(all_trafficlight_actors, lane_marking_dict, carla_map):
        for actor in all_trafficlight_actors:
            base_transform = actor.get_transform()
            tv_loc = actor.trigger_volume.location
            tv_ext = actor.trigger_volume.extent
            x_values = np.arange(-0.9 * tv_ext.x, 0.9 * tv_ext.x, 1.0)
            area = []
            for x in x_values:
                point_location = base_transform.transform(tv_loc + carla.Location(x=x)) 
                area.append(point_location)
            ini_wps = []
            for pt in area:
                wpx = carla_map.get_waypoint(pt)
                # As x_values are arranged in order, only the last one has to be checked
                if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
                    ini_wps.append(wpx)
            
            close2junction_points = []
            littlefar2junction_points = []
            for wpx in ini_wps:
                while not wpx.is_intersection:
                    next_wp = wpx.next(0.5)
                    if not next_wp:
                        break
                    next_wp = next_wp[0]
                    if next_wp and not next_wp.is_intersection:
                        wpx = next_wp
                    else:
                        break
                vec_forward = wpx.transform.get_forward_vector()
                vec_right = carla.Vector3D(x=-vec_forward.y, y=vec_forward.x, z=0) # 2D

                loc_left = wpx.transform.location - 0.4 * wpx.lane_width * vec_right
                loc_right = wpx.transform.location + 0.4 * wpx.lane_width * vec_right
                close2junction_points.append([loc_left.x, loc_left.y, loc_left.z])
                close2junction_points.append([loc_right.x, loc_right.y, loc_right.z])
                
                try:
                    loc_far_left = wpx.previous(0.5)[0].transform.location - 0.4 * wpx.lane_width * vec_right
                    loc_far_right = wpx.previous(0.5)[0].transform.location + 0.4 * wpx.lane_width * vec_right
                except Exception:
                    continue
                
                littlefar2junction_points.append([loc_far_left.x, loc_far_left.y, loc_far_left.z])
                littlefar2junction_points.append([loc_far_right.x, loc_far_right.y, loc_far_right.z])
                
            traffic_light_points = close2junction_points + littlefar2junction_points[::-1]
            trigger_volume_wp = carla_map.get_waypoint(base_transform.transform(tv_loc))
            actor_loc = actor.get_location()
            actor_loc_points = [actor_loc.x, actor_loc.y, actor_loc.z]
            TriggerVolumeGettor.insert_point_into_dict(lane_marking_dict, traffic_light_points, trigger_volume_wp.road_id, actor_loc_points, Volume_Type='TrafficLight')
        pass
    
    pass
t = 0
class LankMarkingGettor(object):

    '''
        structure of lane_marking_dict:
        {
            road_id_0: {
                lane_id_0: [{'Points': [((location.x,y,z) array, (rotation.roll, pitch, yaw))], 'Type': 'lane_marking_type', 'Color':'color', 'Topology':[neighbor array]}, ...]
                ... ...
                'Trigger_Volumes': [{'Points': [(location.x,y,z) array], 'Type': 'trigger volume type', 'ParentActor_Location': (location.x,y,z)}]
            }
            ... ...
        }
        "location array" is an array formed as (location_x, location_y, location_z) ...
        'lane_marking_type' is string of landmarking type, can be 'Broken', 'Solid', 'SolidSolid', 'Other', 'NONE', etc. 
        'color' is string of landmarking color, can be 'Blue', 'White', 'Yellow',  etc. 
         neighbor array contains the ('road_id', 'lane_id') of the current landmarking adjacent to, it is directional. 
         and if current 'Type' == 'Center', there will exist a 'TopologyType' key which record the current lane's topology status. 
         if there exist a trigger volume in current road, key 'Trigger_Volumes' will be added into dict 
         where 'Points' refer to the vertexs location array, 'Type' can be 'StopSign' or 'TrafficLight'
         'ParentActor_Location' is the location of parent actor relevant to this trigger volume. 
    '''
    
    @staticmethod
    def get_lanemarkings(carla_map, lane_marking_dict={}, pixels_per_meter=2, precision=0.05):
        
        topology = [x[0] for x in carla_map.get_topology()]
        topology = sorted(topology, key=lambda w: w.road_id)
        
        for waypoint in topology:
            waypoints = [waypoint]
            # Generate waypoints of a road id. Stop when road id differs
            nxt = waypoint.next(precision)
            if len(nxt) > 0:
                nxt = nxt[0]
                temp_wp = nxt
                while nxt.road_id == waypoint.road_id:
                    waypoints.append(nxt)
                    nxt = nxt.next(precision)
                    if len(nxt) > 0:
                        nxt = nxt[0]
                    else:
                        break
            print("current road id: ", waypoint.road_id)
            print("lane id:", waypoint.lane_id)
            LankMarkingGettor.get_lane_markings_two_side(waypoints, lane_marking_dict)

    @staticmethod
    def get_lane_markings_two_side(waypoints, lane_marking_dict):
        left_lane_marking_list = []
        right_lane_marking_list = []
        
        center_lane_list = []
        center_lane_wps = []
        
        left_previous_lane_marking_type = 1
        left_previous_lane_marking_color = 1
        right_previous_lane_marking_type = 1
        right_previous_lane_marking_color = 1
        
        center_previous_lane_id = waypoints[0].lane_id
        
        for waypoint in waypoints:
            flag = False
            if waypoint.lane_id != center_previous_lane_id:
                if len(center_lane_list) > 1:
                    if waypoint.road_id not in lane_marking_dict:
                        lane_marking_dict[waypoint.road_id] = {}
                        status = check_waypoints_status(center_lane_wps)
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
                    elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                        status = check_waypoints_status(center_lane_wps)
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
                    else:
                        status = check_waypoints_status(center_lane_wps)
                        lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
                flag = True
                center_lane_list = []
                center_lane_wps = []
            left_lane_marking = waypoint.left_lane_marking
            if left_lane_marking.type != left_previous_lane_marking_type or\
                left_lane_marking.color != left_previous_lane_marking_color or flag:
                    if len(left_lane_marking_list) > 1:
                        connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
                        candidate_dict = {'Points': left_lane_marking_list[:], 'Type': str(left_previous_lane_marking_type), 'Color': str(left_previous_lane_marking_color), 'Topology': connect_to[:]}
                        if waypoint.road_id not in lane_marking_dict:
                            lane_marking_dict[waypoint.road_id] = {}
                            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                        elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                        else:
                            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
                        left_lane_marking_list = []
            right_lane_marking = waypoint.right_lane_marking
            if right_lane_marking.type != right_previous_lane_marking_type or\
                right_lane_marking.color != right_previous_lane_marking_color or flag:
                    if len(right_lane_marking_list) > 1:
                        connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
                        candidate_dict = {'Points': right_lane_marking_list[:], 'Type': str(right_previous_lane_marking_type), 'Color': str(right_previous_lane_marking_color), 'Topology': connect_to[:]}
                        if waypoint.road_id not in lane_marking_dict:
                            lane_marking_dict[waypoint.road_id] = {}
                            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                        elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                            lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
                        else:
                            lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
                        right_lane_marking_list = []
                        
            center_lane_list.append((*LankMarkingGettor.get_lateral_shifted_transform(waypoint.transform, 0), waypoint.is_junction))
            center_lane_wps.append(waypoint)
            
            left_lane_marking_list.append(LankMarkingGettor.get_lateral_shifted_transform(waypoint.transform, -0.5*waypoint.lane_width))
            
            right_lane_marking_list.append(LankMarkingGettor.get_lateral_shifted_transform(waypoint.transform, 0.5*waypoint.lane_width))
            
            left_previous_lane_marking_type = left_lane_marking.type
            left_previous_lane_marking_color = left_lane_marking.color
            right_previous_lane_marking_type = right_lane_marking.type
            right_previous_lane_marking_color = right_lane_marking.color
            center_previous_lane_id = waypoint.lane_id
        
        if len(left_lane_marking_list) > 1:
            connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
            candidate_dict = {'Points': left_lane_marking_list[:], 'Type': str(left_lane_marking.type), 'Color': str(left_previous_lane_marking_color), 'Topology': connect_to[:]}
            if waypoint.road_id not in lane_marking_dict:
                lane_marking_dict[waypoint.road_id] = {}
                lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
            elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
            else:
                lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
            left_lane_marking_list = []
        if len(right_lane_marking_list) > 1:
            connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
            candidate_dict = {'Points': right_lane_marking_list[:], 'Type': str(right_lane_marking.type), 'Color': str(right_previous_lane_marking_color), 'Topology': connect_to[:]}
            if waypoint.road_id not in lane_marking_dict:
                lane_marking_dict[waypoint.road_id] = {}
                lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
            elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                lane_marking_dict[waypoint.road_id][center_previous_lane_id] = [candidate_dict]
            else:
                lane_marking_dict[waypoint.road_id][center_previous_lane_id].append(candidate_dict)
            right_lane_marking_list = []
        if len(center_lane_list) > 0:
            if waypoint.road_id not in lane_marking_dict:
                lane_marking_dict[waypoint.road_id] = {}
                status = check_waypoints_status(center_lane_wps)
                lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
                lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
            elif center_previous_lane_id not in lane_marking_dict[waypoint.road_id]:
                status = check_waypoints_status(center_lane_wps)
                lane_marking_dict[waypoint.road_id][center_previous_lane_id] = []
                lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
            else:
                status = check_waypoints_status(center_lane_wps)
                lane_marking_dict[waypoint.road_id][center_previous_lane_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:], 'TopologyType': status, 'Left':(center_lane_wps[-1].get_left_lane().road_id if center_lane_wps[-1].get_left_lane() else None, center_lane_wps[-1].get_left_lane().lane_id if center_lane_wps[-1].get_left_lane() else None), 'Right':(center_lane_wps[-1].get_right_lane().road_id if center_lane_wps[-1].get_right_lane() else None, center_lane_wps[-1].get_right_lane().lane_id if center_lane_wps[-1].get_right_lane() else None)})
            
    @staticmethod
    def get_connected_road_id(waypoint):
        next_waypoint = waypoint.next(0.05)
        if next_waypoint is None:
            return [None]
        else:
            return [(w.road_id, w.lane_id) for w in next_waypoint if w.lane_type == carla.LaneType.Driving]
    
    @staticmethod
    def insert_element_into_dict(id, element, lane_marking_dict):
        if id not in lane_marking_dict:
            lane_marking_dict[id] = []
            lane_marking_dict[id].append(element)
        else:
            lane_marking_dict[id].append(element)
    
    @staticmethod   
    def get_lateral_shifted_transform(transform, shift):
        right_vector = transform.get_right_vector()
        x_offset = right_vector.x * shift
        y_offset = right_vector.y * shift
        z_offset = right_vector.z * shift
        x = transform.location.x + x_offset
        y = transform.location.y + y_offset
        z = transform.location.z + z_offset
        roll = transform.rotation.roll
        pitch = transform.rotation.pitch
        yaw = transform.rotation.yaw
        return ((x, y, z), (roll, pitch, yaw))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_dir', default='/maps')
    parser.add_argument('--carla_town', default='Town12')

    args = parser.parse_args()
    carla_town = args.carla_town

    client = carla.Client('localhost', 2000)
    client.set_timeout(300)
    world = client.load_world(carla_town)
    print("******** sucessfully load the town:", carla_town, " ********")
    carla_map = world.get_map()

    lane_marking_dict = {}
    LankMarkingGettor.get_lanemarkings(world.get_map(), lane_marking_dict)
    print("****** get all lanemarkings ******")
    
    all_actors = world.get_actors()
    all_stop_sign_actors = []
    all_traffic_light_actors = []
    for actor in all_actors:
        if 'traffic.stop' in actor.type_id:
            all_stop_sign_actors.append(actor)
        if 'traffic_light' in actor.type_id:
            all_traffic_light_actors.append(actor)
    
    print("Getting all trigger volumes ...")
    TriggerVolumeGettor.get_stop_sign_trigger_volume(all_stop_sign_actors, lane_marking_dict, carla_map)
    TriggerVolumeGettor.get_traffic_light_trigger_volume(all_traffic_light_actors, lane_marking_dict, carla_map)
    print("******* Have get all trigger volumes ! *********")
    
    arr = np.array(list(lane_marking_dict.items()), dtype=object)
    np.savez_compressed(args.save_dir+"/"+args.carla_town+"_HD_map.npz", arr=arr)
