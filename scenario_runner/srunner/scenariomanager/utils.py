import numpy as np
import cv2
from matplotlib import cm
import math
import open3d as o3d

WINDOW_HEIGHT = 900
WINDOW_WIDTH = 1600

DIS_CAR_SAVE = 50
DIS_WALKER_SAVE = 50
DIS_SIGN_SAVE = 50
DIS_LIGHT_SAVE = 50

edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

carla_bbox_edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
    (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
    (0, 4), (1, 5), (2, 6), (3, 7)   # Side edges connecting top and bottom faces
]

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses

SEM_SEG_LABEL_COLORS = {
         0 : (  0,   0,   0),   # unlabeled      
            # cityscape
         1 : (128,  64, 128),   # road           
         2 : (244,  35, 232),   # sidewalk       
         3 : ( 70,  70,  70),   # building       
         4 : (102, 102, 156),   # wall           
         5 : (190, 153, 153),   # fence          
         6 : (153, 153, 153),   # pole           
         7 : (250, 170,  30),   # traffic light  
         8 : (220, 220,   0),   # traffic sign   
         9 : (107, 142,  35),   # vegetation     
        10 : (152, 251, 152),   # terrain        
        11 : ( 70, 130, 180),   # sky            
        12 : (220,  20,  60),   # pedestrian     
        13 : (255,   0,   0),   # rider          
        14 : (  0,   0, 142),   # Car            
        15 : (  0,   0,  70),   # truck          
        16 : (  0,  60, 100),   # bus            
        17 : (  0,  80, 100),   # train          
        18 : (  0,   0, 230),   # motorcycle     
        19 : (119,  11,  32),   # bicycle        
            # custom
        20 : (110, 190, 160),   # static    
        21 : (170, 120,  50),   # dynamic   
        22 : ( 55,  90,  80),   # other     
        23 : ( 45,  60, 150),   # water     
        24 : (157, 234,  50),   # road line 
        25 : ( 81,   0,  81),   # ground    
        26 : (150, 100, 100),   # bridge    
        27 : (230, 150, 140),   # rail track
        28 : (180, 165, 180),   # guard rail
}

uniad_class_names = [
    'car', 'truck', 'trailer', 'bus', 'construction_vehicle', 'bicycle',
    'motorcycle', 'pedestrian', 'traffic_cone', 'barrier'
]

TYPE_ID_MAP = {
    #=================vehicle=================
    # bicycle
    'vehicle.bh.crossbike': 'bicycle',
    # car
    "vehicle.chevrolet.impala": 'car',
    "vehicle.diamondback.century": 'car',
    "vehicle.dodge.charger_2020": 'car',
    "vehicle.lincoln.mkz_2020": 'car',
    "vehicle.mini.cooper_s_2021": 'car',
    "vehicle.mercedes.coupe_2020": 'car',
    "vehicle.ford.mustang": 'car',
    "vehicle.nissan.patrol_2021": 'car',
    "vehicle.audi.tt": 'car',
    "vehicle.lincoln.mkz_2017": 'car',
    #=========================================

    #=================traffic sign============
    # traffic.speed_limit
    "traffic.speed_limit.30": 'speed_limit',
    "traffic.speed_limit.40": 'speed_limit',
    "traffic.speed_limit.50": 'speed_limit',
    # traffic.traffic_light
    "traffic.traffic_light": 'traffic_light',
    # traffic.stop
    "traffic.stop": 'stop',
    #=========================================
}

def calc_projected_2d_bbox(vertices_pos2d):
    """ Takes in all vertices in pixel projection and calculates min and max of all x and y coordinates.
        Returns left top, right bottom pixel coordinates for the 2d bounding box as a list of four values.
        Note that vertices_pos2d contains a list of (y_pos2d, x_pos2d) tuples, or None
    """
    x_coords = vertices_pos2d[:, 0]
    y_coords = vertices_pos2d[:, 1]
    min_x, max_x = np.min(x_coords), np.max(x_coords)
    min_y, max_y = np.min(y_coords), np.max(y_coords)
    return [min_x, min_y, max_x, max_y]

def calculate_occlusion(bbox, point_depth, agent, depth_map):
    """Calculate the occlusion value of a 2D bounding box.
    Iterate through each point (pixel) in the bounding box and declare it occluded only
    if the 4 surroinding points (pixels) are closer to the camera (by using the help of depth map)
    than the actual distance to the middle of the 3D bounding boxe and some margin (the extent of the object)
    """
    bbox_3d_mid = np.mean(point_depth)
    min_x, min_y, max_x, max_y = calc_projected_2d_bbox(bbox)
    height, width, length = agent.bounding_box.extent.z, agent.bounding_box.extent.x, agent.bounding_box.extent.y

    #depth_margin should depend on the rotation of the object but this solution works fine
    depth_margin = np.max([2 * width, 2 * length])
    is_occluded = []

    for x in range(int(min_x), int(max_x)):
        for y in range(int(min_y), int(max_y)):
            is_occluded.append(point_is_occluded(
                (y, x), bbox_3d_mid - depth_margin, depth_map))

    occlusion = ((float(np.sum(is_occluded))) / ((max_x-min_x) * (max_y-min_y)))
    #discretize the 0–1 occlusion value into KITTI’s {0,1,2,3} labels by equally dividing the interval into 4 parts
    # occlusion = np.digitize(occlusion, bins=[0.25, 0.50, 0.75])
    return occlusion

def calculate_occlusion_vectorized(bbox, point_depth, extent, depth_map):
    """Calculate the occlusion value of a 2D bounding box.
    Iterate through each point (pixel) in the bounding box and declare it occluded only
    if the 4 surroinding points (pixels) are closer to the camera (by using the help of depth map)
    than the actual distance to the middle of the 3D bounding boxe and some margin (the extent of the object)
    """
    bbox_3d_mid = np.mean(point_depth)
    min_x, min_y, max_x, max_y = calc_projected_2d_bbox(bbox)
    height, width, length = extent[2], extent[0], extent[1]
    depth_margin = np.max([2 * width, 2 * length])
    count_num = (max_x - min_x) * (max_y - min_y)
    if count_num > 10000:
        p = 100 / count_num
    elif count_num > 1000:
        p = 100 / count_num
    elif count_num > 100:
        p = 100 / count_num
    else:
        p = 1
    sample_step_approx = int(np.sqrt(1/p))

    # x, y = np.meshgrid(np.arange(min_x, max_x), np.arange(min_y, max_y))
    x, y = np.meshgrid(np.arange(min_x, max_x, sample_step_approx), np.arange(min_y, max_y, sample_step_approx))
    points = np.stack((y.flatten(), x.flatten()), axis=1)
    is_occluded_array = point_is_occluded_single(points, bbox_3d_mid - depth_margin, depth_map)
    occlusion = is_occluded_array.mean()
    #discretize the 0–1 occlusion value into KITTI’s {0,1,2,3} labels by equally dividing the interval into 4 parts
    # occlusion = np.digitize(occlusion, bins=[0.25, 0.50, 0.75])
    return occlusion

def calc_bbox2d_area(bbox_2d):
    """ Calculate the area of the given 2d bbox
    Input is assumed to be xmin, ymin, xmax, ymax tuple 
    """
    xmin, ymin, xmax, ymax = bbox_2d
    return (ymax - ymin) * (xmax - xmin)

def calculate_truncation(uncropped_bbox, cropped_bbox):
    "Calculate how much of the object's 2D uncropped bounding box is outside the image boundary"

    area_cropped = calc_bbox2d_area(cropped_bbox)
    area_uncropped = calc_bbox2d_area(uncropped_bbox)
    truncation = 1.0 - float(area_cropped / area_uncropped)
    return truncation

def crop_boxes_in_canvas(cam_bboxes):
    neg_x_inds = np.where(cam_bboxes[:, 0] < 0)[0]
    out_x_inds = np.where(cam_bboxes[:, 0] > WINDOW_WIDTH)[0]
    neg_y_inds = np.where(cam_bboxes[:, 1] < 0)[0]
    out_y_inds = np.where(cam_bboxes[:, 1] > WINDOW_HEIGHT)[0]
    cam_bboxes[neg_x_inds, 0] = 0
    cam_bboxes[out_x_inds, 0] = WINDOW_HEIGHT
    cam_bboxes[neg_y_inds, 1] = 0
    cam_bboxes[out_y_inds, 1] = WINDOW_WIDTH
    return cam_bboxes

def point_is_occluded(point, vertex_depth, depth_map):
    """ Checks whether or not the four pixels directly around the given point has less depth than the given vertex depth
        If True, this means that the point is occluded.
    """
    y, x = map(int, point)
    from itertools import product
    neigbours = product((1, -1), repeat=2)
    is_occluded = []
    for dy, dx in neigbours:
        if point_in_canvas_hw((dy+y, dx+x)):
            # If the depth map says the pixel is closer to the camera than the actual vertex
            if depth_map[y+dy, x+dx] < vertex_depth:
                is_occluded.append(True)
            else:
                is_occluded.append(False)
    # Only say point is occluded if all four neighbours are closer to camera than vertex
    return all(is_occluded)

def point_is_occluded_single(points, vertex_depth, depth_map, canvas_shape=(WINDOW_HEIGHT, WINDOW_WIDTH)):
    '''
    Simplified version that checks occlusion based only on the points' own depth
    '''
    points = np.asarray(points).astype(np.int32)
    y, x = points[:, 0], points[:, 1]
        
    valid = (y >= 0) & (y < canvas_shape[0]) & \
            (x >= 0) & (x < canvas_shape[1])
    
    is_occluded = np.zeros(len(points), dtype=bool)
    try:
        is_occluded[valid] = depth_map[y[valid], x[valid]] < vertex_depth
    except:
        pass
    return is_occluded

def point_is_occluded_vectorized(points, vertex_depth, depth_map, canvas_shape=(WINDOW_HEIGHT, WINDOW_WIDTH)):
    '''
    Equivalent to point_is_occluded
    '''
    points = np.asarray(points).astype(np.int32)
    y, x = points[:, 0], points[:, 1]
    
    dy, dx = np.array([1, 1, -1, -1]), np.array([1, -1, 1, -1])
    neighbour_y = y[:, np.newaxis] + dy
    neighbour_x = x[:, np.newaxis] + dx
    
    valid = (neighbour_y >= 0) & (neighbour_y < canvas_shape[0]) & \
            (neighbour_x >= 0) & (neighbour_x < canvas_shape[1])
    
    neighbour_depths = np.full(neighbour_y.shape, np.inf)
    for i in range(4):
        mask = valid[:, i]
        neighbour_depths[mask, i] = depth_map[neighbour_y[mask, i], neighbour_x[mask, i]]
    
    is_occluded = np.logical_and.reduce(neighbour_depths < vertex_depth, axis=1) & np.any(valid, axis=1)
    return is_occluded

def draw_3d_bbox_vertex(image, points):
    for x_2d, y_2d, vertex_color in points:
        cv2.circle(image, (int(x_2d), int(y_2d)), radius=3, color=vertex_color, thickness=1)

def calculate_occlusion_stats(bbox_points, depth, depth_map, max_render_depth):
    """ Draws each vertex in vertices_pos2d if it is in front of the camera
        The color is based on whether the object is occluded or not.
        Returns the number of visible vertices and the number of vertices outside the camera.
    """
    num_visible_vertices = 0
    num_invisible_vertices = 0
    num_vertices_outside_camera = 0
    points = []

    for i in range(len(bbox_points)):
        x_2d = bbox_points[i][0]
        y_2d = bbox_points[i][1]
        point_depth = depth[i]

        # if the point is in front of the camera but not too far away
        if max_render_depth > point_depth > 0 and point_in_canvas_hw((y_2d, x_2d)):
            #is_occluded_v = point_is_occluded_vectorized([[y_2d, x_2d]], point_depth, depth_map)
            is_occluded = point_is_occluded(
                (y_2d, x_2d), point_depth, depth_map)
                
            if is_occluded:
                vertex_color = (0,0,255) # bgr, red
                num_invisible_vertices += 1
            else:
                num_visible_vertices += 1
                vertex_color = (0,255,0) # bgr, green
            points.append((x_2d, y_2d, vertex_color))
        else:
            num_vertices_outside_camera += 1
    return num_visible_vertices, num_invisible_vertices, num_vertices_outside_camera, points

def get_intrinsic_matrix(camera):

    width = int(camera.attributes['image_size_x'])
    height = int(camera.attributes['image_size_y'])
    fov = float(camera.attributes['fov'])

    k = np.identity(3)
    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    k[0, 0] = k[1, 1] = width / (2.0 * np.tan(fov * np.pi / 360.0))

    return k

def get_image_point(loc, K, w2c):
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc[0], loc[1], loc[2], 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth componebonent also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    depth = point_camera[2]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]
    
    return point_img[0:2], depth

def point_in_canvas_hw(pos):
    """Return true if point is in canvas"""
    if (pos[0] >= 0) and (pos[0] < WINDOW_HEIGHT) and (pos[1] >= 0) and (pos[1] < WINDOW_WIDTH):
        return True
    return False

def point_in_canvas_wh(pos):
    """Return true if point is in canvas"""
    if (pos[0] >= 0) and (pos[0] < WINDOW_WIDTH) and (pos[1] >= 0) and (pos[1] < WINDOW_HEIGHT):
        return True
    return False

def build_projection_matrix(w, h, fov, is_behind_camera=False):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)

    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal

    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def rotate_3d(vector, theta):
    theta = np.radians(theta)
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1]])

    v_rotated = np.dot(R, vector)
    return np.array([v_rotated[0], v_rotated[1], v_rotated[2]])

def normalize_angle_degree(x):
    x = x % 360.0
    if x > 180.0:
        x -= 360.0
    return x


def align_lidar(lidar, translation, yaw):
    """
    Translates and rotates a LiDAR into a new coordinate system.
    Rotation is inverse to translation and yaw
    :param lidar: numpy LiDAR point cloud (N,3)
    :param translation: translations in meters
    :param yaw: yaw angle in radians
    :return: numpy LiDAR point cloud in the new coordinate system.
    """

    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])

    aligned_lidar = (rotation_matrix.T @ (lidar - translation).T).T

    return aligned_lidar

def convert_depth(data):
    """
    Computes the normalized depth from a CARLA depth map.
    """
    data = data.astype(np.float16)

    normalized = np.dot(data, [65536.0, 256.0, 1.0])
    normalized /= (256 * 256 * 256 - 1)
    return normalized * 1000

def get_relative_transform(ego_matrix, vehicle_matrix):
    """
    Returns the position of the vehicle matrix in the ego coordinate system.
    :param ego_matrix: ndarray 4x4 Matrix of the ego vehicle in global
    coordinates
    :param vehicle_matrix: ndarray 4x4 Matrix of another actor in global
    coordinates
    :return: ndarray position of the other vehicle in the ego coordinate system
    """
    relative_pos = vehicle_matrix[:3, 3] - ego_matrix[:3, 3]
    rot = ego_matrix[:3, :3].T
    relative_pos = rot @ relative_pos

    return relative_pos

def normalize_angle(x):
    x = x % (2 * np.pi)  # force in range [0, 2 pi)
    if x > np.pi:  # move to [-pi, pi)
        x -= 2 * np.pi
    return x

def build_skeleton(ped, sk_links):

    ######## get the pedestrian skeleton #########
    bones = ped.get_bones()

    # list where we will store the lines we will project
    # onto the camera output
    lines_3d = []

    # cycle through the bone pairs in skeleton.txt and retrieve the joint positions
    for link in sk_links[1:]:

        # get the roots of the two bones to be joined
        bone_transform_1 = next(filter(lambda b: b.name == link[0], bones.bone_transforms), None)
        bone_transform_2 = next(filter(lambda b: b.name == link[1], bones.bone_transforms), None)

        # some bone names aren't matched
        if bone_transform_1 is not None and bone_transform_2 is not None:
            lines_3d.append([(bone_transform_1.world.location.x, bone_transform_1.world.location.y, bone_transform_1.world.location.z), 
                             (bone_transform_2.world.location.x, bone_transform_2.world.location.y, bone_transform_2.world.location.z)]
                            )
    return lines_3d

def get_walker_bones(npc):
    """
    Returns:
        bones_dict: {
            bone_name: {
                'location': [x, y, z],
                'rotation': [pitch, roll, yaw]
            }
        }
    """
    bones = npc.get_bones()
    bones_dict = {}

    for bone in bones.bone_transforms:
        loc = bone.world.location
        rot = bone.world.rotation
        bones_dict[bone.name] = {
            'location': [loc.x, loc.y, loc.z],
            'rotation': [rot.pitch, rot.roll, rot.yaw]
        }

    return bones_dict

def get_center_and_extent(verts):
    sum_x = sum_y = sum_z = 0
    max_x = max_y = max_z = float('-inf')
    min_x = min_y = min_z = float('inf')

    for pos in verts:
        sum_x += pos.x
        sum_y += pos.y
        sum_z += pos.z
        
        max_x = max(max_x, pos.x)
        max_y = max(max_y, pos.y)
        max_z = max(max_z, pos.z)
        
        min_x = min(min_x, pos.x)
        min_y = min(min_y, pos.y)
        min_z = min(min_z, pos.z)
    
    center = (sum_x / 8, sum_y / 8, sum_z / 8)
    
    extent = ((max_x - min_x)/2, (max_y - min_y)/2, (max_z - min_z)/2)
    return center, extent

def get_forward_vector(yaw):
    yaw_rad = math.radians(yaw)
    
    x = math.cos(yaw_rad)
    y = math.sin(yaw_rad)
    
    z = 0
    return np.array([x, y, z])

def calculate_cube_vertices(center, extent):
    cx, cy, cz = center
    x, y, z = extent
    vertices = [
        (cx + x, cy + y, cz + z),
        (cx + x, cy + y, cz - z),
        (cx + x, cy - y, cz + z),
        (cx + x, cy - y, cz - z),
        (cx - x, cy + y, cz + z),
        (cx - x, cy + y, cz - z),
        (cx - x, cy - y, cz + z),
        (cx - x, cy - y, cz - z)
    ]
    return vertices


def calculate_cube_vertices_2(center, extent):
    cx, cy, cz = center.x,  center.y,  center.z
    x, y, z = extent.x, extent.y, extent.z
    vertices = [
        (cx + x, cy + y, cz + z),
        (cx + x, cy + y, cz - z),
        (cx + x, cy - y, cz + z),
        (cx + x, cy - y, cz - z),
        (cx - x, cy + y, cz + z),
        (cx - x, cy + y, cz - z),
        (cx - x, cy - y, cz + z),
        (cx - x, cy - y, cz - z)
    ]
    return vertices

def draw_dashed_line(img, start_point, end_point, color, thickness=1, dash_length=5):
    """
    Draw a dashed line on the picture
    
    Params: 
    - img: image to draw ths line on.
    - start_point: starting point, (x, y).
    - end_point: ending point, (x, y).
    - color: color of the line, (B, G, R).
    - thickness: thickness of the line.
    - dash_length: every dash's length
    """
    # calculate whole length
    d = np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)
    dx = (end_point[0] - start_point[0]) / d
    dy = (end_point[1] - start_point[1]) / d

    x, y = start_point[0], start_point[1]

    while d >= dash_length:
        # calculate the end of the next section
        x_end = x + dx * dash_length
        y_end = y + dy * dash_length
        cv2.line(img, (int(x), int(y)), (int(x_end), int(y_end)), color, thickness)

        # update starting point and remaining length
        x = x_end + dx * dash_length
        y = y_end + dy * dash_length
        d -= 2 * dash_length

def get_matrix(location, rotation):
    """
    Creates matrix from carla transform.
    """
    pitch, roll, yaw = rotation
    x, y, z = location
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix

def euler_to_rotation_matrix(pitch, roll, yaw):
    # Y-X-Z order, pitch, roll, yaw
    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    # rotation order: pitch first, roll next, finally yaw
    return np.dot(Rz_yaw, np.dot(Rx_roll, Ry_pitch))

def world_to_ego_no(point_world, ego_location, ego_rotation):
    rotation_matrix = euler_to_rotation_matrix(np.radians(ego_rotation[0]),
                                               np.radians(ego_rotation[1]),
                                               np.radians(ego_rotation[2]))
    
    point_relative = np.array(point_world) - np.array(ego_location)
    point = np.dot(rotation_matrix, point_relative)
    # (x, y ,z) -> (y, -x, z)
    point = [point[0], -point[1], point[2]]
    return point

def world_to_ego(point_world, w2e):
    point_world = np.array([point_world[0], point_world[1], point_world[2], 1])
    point_ego = np.dot(w2e, point_world)
    point_ego = [point_ego[1], -point_ego[0], point_ego[2]]
    return point_ego

def world_to_lidar(point_world, w2l):
    point_world = np.array([point_world[0], point_world[1], point_world[2], 1])
    point_lidar = np.dot(w2l, point_world)
    return point_lidar