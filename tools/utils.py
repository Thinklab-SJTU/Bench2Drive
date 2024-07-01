import numpy as np
import cv2
import math

WINDOW_HEIGHT = 900
WINDOW_WIDTH = 1600

DIS_CAR_SAVE = 100
DIS_WALKER_SAVE = 100
DIS_SIGN_SAVE = 100
DIS_LIGHT_SAVE = 100

edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

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

def point_in_canvas_wh(pos):
    """Return true if point is in canvas"""
    if (pos[0] >= 0) and (pos[0] < WINDOW_WIDTH) and (pos[1] >= 0) and (pos[1] < WINDOW_HEIGHT):
        return True
    return False

def get_forward_vector(yaw):
    # Convert the yaw angle from degrees to radians
    yaw_rad = math.radians(yaw)
    # Calculate the X and Y components of the forward vector (in a left-handed coordinate system with Z-axis upwards)
    # Note: In a left-handed coordinate system, the positive Y direction could correspond to either forward or backward, depending on the specific application scenario
    x = math.cos(yaw_rad)
    y = math.sin(yaw_rad)
    # On a horizontal plane, the Z component of the forward vector is 0
    z = 0
    return np.array([x, y, z])

def calculate_cube_vertices(center, extent):
    if isinstance(center, list):
        cx, cy, cz = center
        x, y, z = extent
    else:
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
    Draw a dashed line on an image.
    Arguments:
    - img: The image on which to draw the dashed line.
    - start_point: The starting point of the dashed line, in the format (x, y).
    - end_point: The ending point of the dashed line, in the format (x, y).
    - color: The color of the dashed line, in the format (B, G, R).
    - thickness: The thickness of the line.
    - dash_length: The length of each dash segment in the dashed line.
    """
    # Calculate total length
    d = np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)
    dx = (end_point[0] - start_point[0]) / d
    dy = (end_point[1] - start_point[1]) / d

    x, y = start_point[0], start_point[1]

    while d >= dash_length:
        # Calculate the end point of the next segment
        x_end = x + dx * dash_length
        y_end = y + dy * dash_length
        cv2.line(img, (int(x), int(y)), (int(x_end), int(y_end)), color, thickness)

        # Update starting point and remaining length
        x = x_end + dx * dash_length
        y = y_end + dy * dash_length
        d -= 2 * dash_length

def world_to_ego(point_world, w2e):
    point_world = np.array([point_world[0], point_world[1], point_world[2], 1])
    point_ego = np.dot(w2e, point_world)
    point_ego = [point_ego[1], -point_ego[0], point_ego[2]]
    return point_ego

def vector_angle(v1, v2):
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
    angle_radians = np.arccos(cos_theta)
    angle_degrees = np.degrees(angle_radians)
    return angle_degrees

def get_weather_id(weather_conditions):
    from xml.etree import ElementTree as ET
    tree = ET.parse('./leaderboard/data/weather.xml')
    root = tree.getroot()
    def conditions_match(weather, conditions):
        for (key, value) in weather:
            if key == 'route_percentage' : continue
            if str(conditions[key]) != value:
                return False
        return True
    for case in root.findall('case'):
        weather = case[0].items()
        if conditions_match(weather, weather_conditions):
            return case.items()[0][1]
    return None

def compute_2d_distance(loc1, loc2):
    return math.sqrt((loc1.x-loc2.x)**2+(loc1.y-loc2.y)**2)

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