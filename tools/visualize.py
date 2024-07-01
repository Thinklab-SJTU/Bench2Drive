import json
import gzip
import os
import numpy as np
import cv2
import pathlib
import random
import laspy
import matplotlib.cm as cm
from tqdm import trange
from utils import get_image_point, point_in_canvas_wh, edges, world_to_ego, get_forward_vector, calculate_cube_vertices, draw_dashed_line, vector_angle, get_weather_id

def visualize_data(file_path, map_path, vis_bbox=True,  vis_top_down=True, vis_road=True, vis_lidar_bev=True, vis_lidar_to_back_image=True, vis_lidar_to_front_image=True, vis_lidar_to_front_left_image=True):
    print(f'file_path={file_path}')
    print(f'map_path={map_path}')

    save_path = pathlib.Path(file_path.replace('v0','v0-vis'))
    (save_path / 'camera' / 'rgb_front_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_front_landmark').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_front_left_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_front_right_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_back_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_back_left_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_back_right_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'camera' / 'rgb_top_down_3d_bbox').mkdir(parents=True, exist_ok=True)
    (save_path / 'lidar'  / 'bev').mkdir(parents=True, exist_ok=True)
    (save_path / 'lidar'  / 'front').mkdir(parents=True, exist_ok=True)
    (save_path / 'lidar'  / 'front_left').mkdir(parents=True, exist_ok=True)
    (save_path / 'lidar'  / 'back').mkdir(parents=True, exist_ok=True)

    cam_map = {
        'CAM_FRONT': 'rgb_front',
        'CAM_FRONT_LEFT': 'rgb_front_left',
        'CAM_FRONT_RIGHT': 'rgb_front_right',
        'CAM_BACK': 'rgb_back', 
        'CAM_BACK_LEFT': 'rgb_back_left', 
        'CAM_BACK_RIGHT': 'rgb_back_right',
        'TOP_DOWN': 'rgb_top_down'
    }

    folder_path = os.path.join(file_path, 'anno')
    file_count = len([name for name in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, name))])
    map_info = dict(np.load(map_path, allow_pickle=True)['arr'])

    for step in trange(file_count):
        with gzip.open(os.path.join(file_path, f'anno/{step:05}.json.gz'), 'rt', encoding='utf-8') as gz_file:
            anno = json.load(gz_file)
        weather_id = get_weather_id(anno['weather'])
        bounding_boxes = anno['bounding_boxes']
        sensors_anno = anno['sensors']
        # ========================== bbox ==========================
        if vis_bbox:            
            for key in ['CAM_FRONT','CAM_FRONT_LEFT','CAM_FRONT_RIGHT','CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT']:
                K = sensors_anno[key]['intrinsic']
                world2cam = sensors_anno[key]['world2cam']
                visulize_img = cv2.imread(os.path.join(file_path, f'camera/{cam_map[key]}/{step:05}.jpg'))
                for npc in bounding_boxes:
                    if npc['class'] == 'ego_vehicle': continue
                    if npc['distance'] > 75: continue
                    if abs(npc['location'][2] - anno['bounding_boxes'][0]['location'][2]) > 10: continue # car in sky and underground
                    if 'vehicle' in npc['class']: # vehicle
                        forward_vec = get_forward_vector(sensors_anno[key]['rotation'][2])
                        ray = np.array(npc['location']) - np.array(sensors_anno[key]['location'])
                        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                        if forward_vec.dot(ray) > 1 and vector_angle(forward_vec, ray)<45:
                            verts = np.array(npc['world_cord'])
                            for edge in edges:
                                p1, p1_depth = get_image_point(verts[edge[0]], K, world2cam)
                                p2, p2_depth = get_image_point(verts[edge[1]],  K, world2cam)
                                draw_dashed_line(visulize_img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), color, 2)
                            cv2.putText(visulize_img, npc['class']+npc['id'], (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
                    else: # sign, light, pedestrians
                        if npc['class'] == 'traffic_sign': 
                            npc['extent'][1] = 0.5 # traffic_sign origin y is too small
                        
                        forward_vec = get_forward_vector(sensors_anno[key]['rotation'][2])
                        ray = np.array(npc['location']) - np.array(sensors_anno[key]['location'])
                        if 'affects_ego' in npc.keys() and str(npc['affects_ego']) == 'True':
                            color = (0, 0, 255)
                        else:
                            color = (255, 255, 255)
                        if forward_vec.dot(ray) > 1 and vector_angle(forward_vec, ray)<45:
                            if 'world_cord' in npc.keys():
                                if 'dirtdebris' in npc['type_id']:
                                    local_verts = calculate_cube_vertices(npc['bbx_loc'], [npc['extent'][1], npc['extent'][0], npc['extent'][2]])
                                    verts = []
                                    for l_v in local_verts:
                                        g_v = np.dot(np.matrix(npc['world2sign']).I, [l_v[0], l_v[1], l_v[2],1])
                                        verts.append(g_v.tolist()[0][:-1])
                                else:
                                    verts = np.array(npc['world_cord'])
                            else:
                                verts = calculate_cube_vertices(npc['center'], npc['extent'])
                            for edge in edges:
                                p1, p1_depth = get_image_point(verts[edge[0]], K, world2cam)
                                p2, p2_depth = get_image_point(verts[edge[1]],  K, world2cam)
                                draw_dashed_line(visulize_img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), color, 2)
                            if 'affects_ego' in npc.keys():
                                cv2.putText(visulize_img, npc['class'], (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
                            else:
                                cv2.putText(visulize_img, npc['class'], (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
                cv2.imwrite(os.path.join(save_path, f'camera/{cam_map[key]}_3d_bbox/{step:05}.jpg'), visulize_img)
        
        if vis_top_down:        
            for key in ['TOP_DOWN']:
                K = sensors_anno[key]['intrinsic']
                world2cam = sensors_anno[key]['world2cam']
                visulize_img = cv2.imread(os.path.join(file_path, f'camera/{cam_map[key]}/{step:05}.jpg'))
                road_points = map_info[anno['bounding_boxes'][0]['road_id']]
                # draw lane
                for r_p in road_points[anno['bounding_boxes'][0]['lane_id']]:
                    road_point = r_p['Points']
                    road_type = r_p['Type']
                    road_color = r_p['Color']
                    road_topology = r_p['Topology']
                    for point in road_point:
                        point = np.array([point[0][0], point[0][1], point[0][2], 1])
                        point_camera = np.dot(world2cam, point)
                        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
                        depth = point_camera[2]
                        point_img = np.dot(K, point_camera)
                        if depth >0:
                            point_img[0] /= point_img[2]
                            point_img[1] /= point_img[2]
                            point_img = point_img[0:2]
                            if point_in_canvas_wh(point_img):
                                if road_color == 'White':
                                    cv2.circle(visulize_img, (int(point_img[0]), int(point_img[1])), radius=1, color=(255, 255, 255), thickness=-1)
                                    if road_type == 'Center':
                                        cv2.circle(visulize_img, (int(point_img[0]), int(point_img[1])), radius=1, color=(0, 255, 0), thickness=-1)
                                else:
                                    cv2.circle(visulize_img, (int(point_img[0]), int(point_img[1])), radius=1, color=(0, 255, 255), thickness=-1)                      
                # draw vehicle
                for npc in bounding_boxes:
                    if 'vehicle' in npc['class']:
                        if abs(npc['location'][2] - anno['bounding_boxes'][0]['location'][2]) > 10: continue # car in sky and underground
                        if npc['class'] == 'ego_vehicle':
                            color = (255, 255, 255, 255)
                        else:
                            color = (255, 0, 0, 255)
                        verts = np.array(npc['world_cord'])
                        p1, p1_depth = get_image_point(verts[0], K, world2cam)
                        p2, p2_depth = get_image_point(verts[2],  K, world2cam)
                        p3, p3_depth = get_image_point(verts[4],  K, world2cam)
                        p4, p4_depth = get_image_point(verts[6],  K, world2cam)
                        points = np.array([p1, p2, p4, p3])
                        height, width = visulize_img.shape[:2]
                        blk = np.zeros((height, width, 4), np.uint8)
                        cv2.fillConvexPoly(blk, np.round(points).astype(np.int32), color)
                        if npc['class'] == 'ego_vehicle':
                            visulize_img = cv2.addWeighted(visulize_img, 1.0, blk[:,:,:3], 1, 1)
                        else:
                            visulize_img = cv2.addWeighted(visulize_img, 1.0, blk[:,:,:3], 0.25, 1)
                        if npc['class'] == 'ego_vehicle':
                            cv2.putText(visulize_img, npc['class'], (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0), 1)
                        else:
                            cv2.putText(visulize_img, npc['class'], (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
                # draw sign
                for npc in bounding_boxes:
                    if abs(npc['location'][2] - anno['bounding_boxes'][0]['location'][2]) > 10: continue # car in sky and underground
                    # traffic_sign
                    if 'traffic_sign' in npc['class']:
                        color = (0, 0, 255, 255)
                        if 'world_cord' in npc.keys():
                            verts = np.array(npc['world_cord'])
                        else:
                            verts = calculate_cube_vertices(npc['center'], npc['extent'])
                        p1, p1_depth = get_image_point(verts[0], K, world2cam)
                        p2, p2_depth = get_image_point(verts[2],  K, world2cam)
                        p3, p3_depth = get_image_point(verts[4],  K, world2cam)
                        p4, p4_depth = get_image_point(verts[6],  K, world2cam)
                        points = np.array([p1, p2, p4, p3])
                        height, width = visulize_img.shape[:2]
                        blk = np.zeros((height, width, 4), np.uint8)
                        cv2.fillConvexPoly(blk, np.round(points).astype(np.int32), color)
                        visulize_img = cv2.addWeighted(visulize_img, 1.0, blk[:,:,:3], 0.25, 1)
                        cv2.putText(visulize_img, npc['class'], (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
                    # traffic_light
                    if 'traffic_light' in npc['class']:
                        color = (255, 0, 0)
                        verts = calculate_cube_vertices(npc['center'], npc['extent'])
                        for edge in edges:
                            p1, p1_depth = get_image_point(verts[edge[0]], K, world2cam)
                            p2, p2_depth = get_image_point(verts[edge[1]],  K, world2cam)
                            cv2.line(visulize_img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), color, 2)
                        cv2.putText(visulize_img, 'traffic_light', (int(p1[0])+2,int(p1[1])+2), cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
                cv2.imwrite(os.path.join(save_path, f'camera/{cam_map[key]}_3d_bbox/{step:05}.jpg'), visulize_img)
        # ==========================================================

        # ========================== road ==========================
        if vis_road:
            key = 'CAM_FRONT'
            K = sensors_anno[key]['intrinsic']
            world2cam = sensors_anno[key]['world2cam']
            road_points = map_info[anno['bounding_boxes'][0]['road_id']]
            road_seg = np.zeros((900, 1600, 3), dtype=np.uint8)
            all_road_topology = set()
            # draw current road
            for r_p in road_points[anno['bounding_boxes'][0]['lane_id']]:
                road_point = r_p['Points']
                road_type = r_p['Type']
                road_color = r_p['Color']
                road_topology = r_p['Topology']
                for r_t in road_topology:
                    all_road_topology.add(r_t)
                for point in road_point:
                    point = np.array([point[0][0], point[0][1], point[0][2], 1])
                    point_camera = np.dot(world2cam, point)
                    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
                    depth = point_camera[2]
                    point_img = np.dot(K, point_camera)
                    if depth >0:
                        point_img[0] /= point_img[2]
                        point_img[1] /= point_img[2]
                        point_img = point_img[0:2]
                        if point_in_canvas_wh(point_img):
                            if road_color == 'White':
                                cv2.circle(road_seg, (int(point_img[0]), int(point_img[1])), radius=1, color=(255, 255, 255), thickness=-1)
                                if road_type == 'Center':
                                    cv2.circle(road_seg, (int(point_img[0]), int(point_img[1])), radius=1, color=(0, 255, 0), thickness=-1)
                            else:
                                cv2.circle(road_seg, (int(point_img[0]), int(point_img[1])), radius=1, color=(0, 255, 255), thickness=-1)
            # draw road topology
            for r_t in all_road_topology:
                road_points = map_info[r_t[0]][r_t[1]]
                for r_p in road_points:
                    road_point = r_p['Points']
                    road_type = r_p['Type']
                    road_color = r_p['Color']
                    road_topology = r_p['Topology']
                    for point in road_point:
                        point = np.array([point[0][0], point[0][1], point[0][2], 1])
                        point_camera = np.dot(world2cam, point)
                        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
                        depth = point_camera[2]
                        point_img = np.dot(K, point_camera)
                        if depth >0:
                            point_img[0] /= point_img[2]
                            point_img[1] /= point_img[2]
                            point_img = point_img[0:2]
                            if point_in_canvas_wh(point_img):
                                if road_color == 'White':
                                    cv2.circle(road_seg, (int(point_img[0]), int(point_img[1])), radius=1, color=(255, 255, 255), thickness=-1)
                                    if road_type == 'Center':
                                        cv2.circle(road_seg, (int(point_img[0]), int(point_img[1])), radius=1, color=(0, 255, 0), thickness=-1)
                                else:
                                    cv2.circle(road_seg, (int(point_img[0]), int(point_img[1])), radius=1, color=(0, 255, 255), thickness=-1)
            cv2.imwrite(os.path.join(save_path, f'camera/{cam_map[key]}_landmark/{step:05}.png'), road_seg)
        # # ===========================================================

        # ========================== lidar to bev =====================
        if vis_lidar_bev:
            lidar_path = os.path.join(file_path, f'lidar/{step:05}.laz')
            lidars = laspy.read(lidar_path).xyz

            lidar_image = np.zeros((900, 1600, 3), dtype=np.uint8)
            header = laspy.LasHeader(point_format=0)  # LARS point format used for storing
            header.offsets = np.min(lidars, axis=0)
            point_precision = 0.001
            header.scales = np.array([point_precision, point_precision, point_precision])
            point_record = laspy.ScaleAwarePointRecord.zeros(lidars.shape[0], header=header)

            point_record.x = lidars[:, 0]
            point_record.y = lidars[:, 1]
            point_record.z = lidars[:, 2]
            # (x, y,z) -> (y, -x, z), the x,y plane of the lidar system is different from that of the ego-car
            point_record.x = lidars[:, 1]
            point_record.y = - lidars[:, 0]
            point_record.z = lidars[:, 2]

            range_x = 85.0
            range_y = 85.0
            range_z = 85.0

            # Normalized point cloud data
            x_normalized = (point_record.x + range_x) / (2 * range_x)
            y_normalized = (point_record.y + range_y) / (2 * range_y)
            z_normalized = (point_record.z + range_z) / (2 * range_z)

            # Convert normalized coordinates to image pixel index
            x_pixels = x_normalized * (lidar_image.shape[1] - 1)
            y_pixels = y_normalized * (lidar_image.shape[0] - 1)

            for x, y, z in zip(x_pixels, y_pixels, z_normalized):
                rgb_color = (255, 255, 255)
                cv2.circle(lidar_image, (int(x), int(y)), radius=2, color=(int(rgb_color[0]), int(rgb_color[1]), int(rgb_color[2])), thickness=-1)

            cv2.imwrite(os.path.join(save_path, f'lidar_bev/{step:05}.png'), lidar_image)
            for npc in bounding_boxes:
                if npc['class'] not in ['vehicle', 'ego_vehicle']: continue
                if abs(npc['location'][2] - anno['bounding_boxes'][0]['location'][2]) > 10: continue # car in sky and underground
                verts = calculate_cube_vertices(npc['center'], npc['extent'])
                verts = np.array(npc['world_cord'])
                # verts[:, 2] = verts[:, 2] - npc['extent'][2] # carla bbox need minus z 
                if npc['class'] == 'ego_vehicle':
                    color = (0, 255, 0)
                else:
                    color = (0, 128, 255)
                for edge in edges:
                    p1 = verts[edge[0]]
                    p1 = world_to_ego(p1, anno['bounding_boxes'][0]['world2ego'])
                    p1_x = (p1[0] + range_x) / (2 * range_x) * (lidar_image.shape[1] - 1)
                    p1_y = (p1[1] + range_y) / (2 * range_y) * (lidar_image.shape[0] - 1)

                    p2 = verts[edge[1]]
                    p2 = world_to_ego(p2, anno['bounding_boxes'][0]['world2ego'])
                    p2_x = (p2[0] + range_x) / (2 * range_x) * (lidar_image.shape[1] - 1)
                    p2_y = (p2[1] + range_y) / (2 * range_y) * (lidar_image.shape[0] - 1)
                    cv2.line(lidar_image, (int(p1_x), int(p1_y)), (int(p2_x), int(p2_y)), color, 2)

            cv2.imwrite(os.path.join(save_path, f'lidar/bev/{step:05}.png'), lidar_image)
        # ===========================================================
        
        # ================ lidar to front image =====================
        if vis_lidar_to_front_image:
            key = 'CAM_FRONT'
            K = sensors_anno[key]['intrinsic']
            visulize_img = cv2.imread(os.path.join(file_path, f'camera/{cam_map[key]}/{step:05}.jpg'))
            ego2cam = np.matrix(sensors_anno[key]['cam2ego']).I.tolist()

            # lidar in ego coordinate
            lidar_path = os.path.join(file_path, f'lidar/{step:05}.laz')
            lidars = laspy.read(lidar_path).xyz

            for lidar in lidars:
                lidar = np.array([lidar[0], lidar[1], lidar[2], 1])
                point_camera = np.dot(ego2cam, lidar)
                point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
                depth = point_camera[2]
                point_img = np.dot(K, point_camera)
                if depth > 0:
                    point_img[0] /= point_img[2]
                    point_img[1] /= point_img[2]
                    point_img = point_img[0:2]
                    if point_in_canvas_wh(point_img):
                        color_scale = min(depth / 80, 1)
                        color = cm.rainbow(color_scale)
                        color = tuple([int(x*255) for x in color[:3]])
                        cv2.circle(visulize_img, (int(point_img[0]), int(point_img[1])), radius=2, color=color, thickness=-1)
            cv2.imwrite(os.path.join(save_path, f'lidar/front/{step:05}_front.png'), visulize_img)
            # ==========================================================

        # ================ lidar to back image =====================
        if vis_lidar_to_back_image:
            key = 'CAM_BACK'
            K = sensors_anno[key]['intrinsic']
            visulize_img = cv2.imread(os.path.join(file_path, f'camera/{cam_map[key]}/{step:05}.jpg'))
            ego2cam = np.matrix(sensors_anno[key]['cam2ego']).I.tolist()

            # lidar in ego coordinate
            lidar_path = os.path.join(file_path, f'lidar/{step:05}.laz')
            lidars = laspy.read(lidar_path).xyz

            for lidar in lidars:
                lidar = np.array([lidar[0], lidar[1], lidar[2], 1])
                point_camera = np.dot(ego2cam, lidar)
                point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
                depth = point_camera[2]
                point_img = np.dot(K, point_camera)
                if depth > 0:
                    point_img[0] /= point_img[2]
                    point_img[1] /= point_img[2]
                    point_img = point_img[0:2]
                    if point_in_canvas_wh(point_img):
                        color_scale = min(depth / 80, 1)
                        color = cm.rainbow(color_scale)
                        color = tuple([int(x*255) for x in color[:3]])
                        cv2.circle(visulize_img, (int(point_img[0]), int(point_img[1])), radius=2, color=color, thickness=-1)
            cv2.imwrite(os.path.join(save_path, f'lidar/back/{step:05}_back.png'), visulize_img)
        # ===========================================================

        # ================ lidar to fomr left image =====================
        if vis_lidar_to_front_left_image:
            key = 'CAM_FRONT_LEFT'
            K = sensors_anno[key]['intrinsic']
            visulize_img = cv2.imread(os.path.join(file_path, f'camera/{cam_map[key]}/{step:05}.jpg'))
            ego2cam = np.matrix(sensors_anno[key]['cam2ego']).I.tolist()

            # lidar in ego coordinate
            lidar_path = os.path.join(file_path, f'lidar/{step:05}.laz')
            lidars = laspy.read(lidar_path).xyz

            for lidar in lidars:
                lidar = np.array([lidar[0], lidar[1], lidar[2], 1])
                point_camera = np.dot(ego2cam, lidar)
                point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
                depth = point_camera[2]
                point_img = np.dot(K, point_camera)
                if depth > 0:
                    point_img[0] /= point_img[2]
                    point_img[1] /= point_img[2]
                    point_img = point_img[0:2]
                    if point_in_canvas_wh(point_img):
                        color_scale = min(depth / 80, 1)
                        color = cm.rainbow(color_scale)
                        color = tuple([int(x*255) for x in color[:3]])
                        cv2.circle(visulize_img, (int(point_img[0]), int(point_img[1])), radius=2, color=color, thickness=-1)
            cv2.imwrite(os.path.join(save_path, f'lidar/front_left/{step:05}_front_left.png'), visulize_img)
        # ===========================================================

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='argparse')
    parser.add_argument('--file_path','-f', type=str)
    parser.add_argument('--map_path','-m', type=str)

    args = parser.parse_args()
    map_path = f'./maps/Town{args.map_path}_HD_map.npz'
    visualize_data(args.file_path, map_path, vis_bbox=True, vis_top_down=True, vis_road=True, vis_lidar_bev=True, vis_lidar_to_back_image=True, vis_lidar_to_front_image=True, vis_lidar_to_front_left_image=True)