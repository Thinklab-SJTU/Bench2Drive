import carla
import numpy as np
import argparse

t = 0
class LankMarkingGettor(object):

    '''
        structure of lane_marking_dict:
        {
            road_id_0: [
                ['Points': [(location.x,y,z) array], 'Type': 'lane_marking_type', 'Color':'color', 'Topology':[neighbor array]]
                    ... ...
            ]
            ... ...
        }
        "location array" is an array formed as (location_x, location_y, location_z) ...
        'lane_marking_type' is string of landmarking type, can be 'Broken', 'Solid', 'SolidSolid', 'Other', 'NONE', etc. 
        'color' is string of landmarking color, can be 'Blue', 'White', 'Yellow',  etc. 
         neighbor array contains the 'road_id' of the current landmarking adjacent to, it is directional. 
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
            LankMarkingGettor.get_lane_markings_two_side(waypoints, lane_marking_dict)

    @staticmethod
    def get_lane_markings_two_side(waypoints, lane_marking_dict):
        global t
        left_lane_marking_list = []
        right_lane_marking_list = []
        
        center_lane_list = []
        
        left_previous_lane_marking_type = 1
        left_previous_lane_marking_color = 1
        right_previous_lane_marking_type = 1
        right_previous_lane_marking_color = 1
        
        for waypoint in waypoints:
            center_lane_list.append(LankMarkingGettor.get_lateral_shifted_transform(waypoint.transform, 0))
            left_lane_marking = waypoint.left_lane_marking
            left_lane_marking_list.append(LankMarkingGettor.get_lateral_shifted_transform(waypoint.transform, -0.5*waypoint.lane_width))
            if left_lane_marking.type != left_previous_lane_marking_type or\
                left_lane_marking.color != left_previous_lane_marking_color:
                    if len(left_lane_marking_list) > 1:
                        connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
                        candidate_dict = {'Points': left_lane_marking_list[:], 'Type': str(left_lane_marking.type), 'Color': str(left_previous_lane_marking_color), 'Topology': connect_to[:]}
                        if waypoint.road_id not in lane_marking_dict:
                            lane_marking_dict[waypoint.road_id] = [candidate_dict]
                        else:
                            lane_marking_dict[waypoint.road_id].append(candidate_dict)
                        left_lane_marking_list = []
                        t += 1
                        print(t)
            right_lane_marking = waypoint.right_lane_marking
            right_lane_marking_list.append(LankMarkingGettor.get_lateral_shifted_transform(waypoint.transform, 0.5*waypoint.lane_width))
            if right_lane_marking.type != right_previous_lane_marking_type or\
                right_lane_marking.color != right_previous_lane_marking_color:
                    if len(right_lane_marking_list) > 1:
                        connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
                        candidate_dict = {'Points': right_lane_marking_list[:], 'Type': str(right_lane_marking.type), 'Color': str(right_previous_lane_marking_color), 'Topology': connect_to[:]}
                        if waypoint.road_id not in lane_marking_dict:
                            lane_marking_dict[waypoint.road_id] = [candidate_dict]
                        else:
                            lane_marking_dict[waypoint.road_id].append(candidate_dict)
                        right_lane_marking_list = []
                        t += 1
                        print(t)
            left_previous_lane_marking_type = left_lane_marking.type
            left_previous_lane_marking_color = left_lane_marking.color
            right_previous_lane_marking_type = right_lane_marking.type
            right_previous_lane_marking_color = right_lane_marking.color
        
        if len(left_lane_marking_list) > 1:
            connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
            candidate_dict = {'Points': left_lane_marking_list[:], 'Type': str(left_lane_marking.type), 'Color': str(left_previous_lane_marking_color), 'Topology': connect_to[:]}
            if waypoint.road_id not in lane_marking_dict:
                lane_marking_dict[waypoint.road_id] = [candidate_dict]
            else:
                lane_marking_dict[waypoint.road_id].append(candidate_dict)
            left_lane_marking_list = []
            t += 1
            print(t)
        if len(right_lane_marking_list) > 1:
            connect_to = LankMarkingGettor.get_connected_road_id(waypoint)
            candidate_dict = {'Points': right_lane_marking_list[:], 'Type': str(right_lane_marking.type), 'Color': str(right_previous_lane_marking_color), 'Topology': connect_to[:]}
            if waypoint.road_id not in lane_marking_dict:
                lane_marking_dict[waypoint.road_id] = [candidate_dict]
            else:
                lane_marking_dict[waypoint.road_id].append(candidate_dict)
            right_lane_marking_list = []
            t += 1
            print(t)
        if waypoint.road_id not in lane_marking_dict:
            lane_marking_dict[waypoint.road_id] = [{'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:]}]
        else:
            lane_marking_dict[waypoint.road_id].append({'Points': center_lane_list[:], 'Type': 'Center', 'Color': 'White', 'Topology': LankMarkingGettor.get_connected_road_id(waypoint)[:]})
            
    @staticmethod
    def get_connected_road_id(waypoint):
        next_waypoint = waypoint.next(0.05)
        if next_waypoint is None:
            return [None]
        else:
            return [w.road_id for w in next_waypoint]
    
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
        return (x, y, z)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_dir', default='./maps')
    parser.add_argument('--carla_map', default='Town13')

    args = parser.parse_args()
    carla_map = args.carla_map

    client = carla.Client('localhost', 30000)
    client.set_timeout(300)
    world = client.load_world(carla_map)
    print("****** sucessfully load the town:", carla_map, " ********")

    lane_marking_dict = {}
    LankMarkingGettor.get_lanemarkings(world.get_map(), lane_marking_dict)
    print("****** get all lanemarkings ******")
    
    arr = np.array(list(lane_marking_dict.items()), dtype=object)
    np.savez_compressed(args.save_dir+"/"+args.carla_map+"_lanemarkings.npz", arr=arr)
    
    
