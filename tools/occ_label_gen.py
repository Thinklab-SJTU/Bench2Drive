#coding=utf-8
import os
import copy
import time
import torch
import pickle
import math

import numpy as np
import open3d as o3d
from tqdm import tqdm
from pdb import set_trace
from scipy.optimize import linear_sum_assignment
from utils import *
from torch.nn import functional as F

class OccDatasetGen():
    def __init__(self, pkl_path, content, frame_num=21):
        self.pkl_path = pkl_path
        self.frame_num = frame_num
        self.key_frame_idx = self.frame_num//2
        self.pad_floor_points = True
        self.similarity_threshold = 0.15
        self.filter_near = True
        self.pc_range_filters = [[-1.5, 1.5, -1.5, 1.5, -1.345 + 0.5, 2.5], [-25.6, 25.6, -25.6, 25.6, -1.5-1.345, -1.345 - 0.5]]
        self.pc_range = [-25.6, 25.6, -25.6, 25.6, -5, 3]
        self.floor_points_range = [-2.5, 2.5, -2.5, 2.5]
        self.voxel_size = [0.4, 0.4, 0.4]
        self.downsample = 12
        self.floor_height = -0.149
        self.pc_range2 = [-25.6, 25.6, -25.6, 25.6, -5, 3]
        self.lidar_coor_space_gen()
        self.floor_points_gen()
        self.content = content
        self.lidar_path_cache = [None for i in range(self.frame_num)]
        self.points_cache = [None for i in range(self.frame_num)]
        self.gt_boxes_cache = [None for i in range(self.frame_num)]
        self.gt_names_cache = [None for i in range(self.frame_num)]
        self.hs_enu_pose_cache = [None for i in range(self.frame_num)]
        self.scene_token_cache = [None for i in range(self.frame_num)]
        self.points_bg_cache = [None for i in range(self.frame_num)]
        self.points_bg_key_cache = [None for i in range(self.frame_num)]
        self.points_objs_cache = [None for i in range(self.frame_num)]
        # self.up_samlple_mask_gen()
        

    def pose_trans(self, source_enu_pose, target_enu_pose):
        trans_mat = np.dot(np.linalg.inv(target_enu_pose), source_enu_pose)
        return trans_mat

    def object_segmentation(self, pcd_points, bboxes, is_Key):
        new_gt_boxes = []
        points_objs = []

        for gt_box in bboxes:
            x, y, z, w, l, h, yaw = gt_box
            z = z + h/2
            yaw = -(yaw +  np.pi/2)
            bbox = [x, y, z, l, w, h, yaw]
            points_obj, _ = remain_points_in_boxes3d(pcd_points, np.array([bbox]))
            points_objs.append(points_obj)
            new_gt_boxes.append(bbox)
        if len(new_gt_boxes) != 0:
            points_bg = remove_points_in_boxes3d(pcd_points, np.array(new_gt_boxes))
        else:
            points_bg = pcd_points
        if self.filter_near and (not is_Key):
            for pc_range_filter in self.pc_range_filters:
                points_bg = self.points_remove_by_range(points_bg, pc_range_filter)
        return points_bg, points_objs

    def rotz(self, t):
        """ Rotation about the z-axis. """
        c = np.cos(t)
        s = np.sin(t)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def obj_aggregate(self):
        '''
        size/points' number/vehcile's yaw/tracklet(velocity's similarity)/whlyaw's similarity/position's similarity
        '''
        gt_boxes_key = copy.deepcopy(self.gt_boxes_cache[self.key_frame_idx])
        points_objs_key = copy.deepcopy(self.points_objs_cache[self.key_frame_idx])
        gt_names_key = copy.deepcopy(self.gt_names_cache[self.key_frame_idx])
        enu_pose_key = copy.deepcopy(self.hs_enu_pose_cache[self.key_frame_idx])
        if len(gt_boxes_key) == 0:
            return np.array([])
        center_points_key = np.array(gt_boxes_key)[:, :3]
        center_points_key = np.expand_dims(center_points_key, 1)
        gt_names_key = np.expand_dims(gt_names_key, 1)

        wlh_key = np.array(gt_boxes_key)[:, 3:6]
        size_key = wlh_key[:,0] * wlh_key[:,1] * wlh_key[:,2]
        size_key = np.expand_dims(size_key, 1)

        points_num_key = np.array([len(points) for points in points_objs_key])
        points_num_key = np.expand_dims(points_num_key, 1)

        yaw_key = np.array(gt_boxes_key)[:, 6] % np.pi
        yaw_key = np.expand_dims(yaw_key, 1)

        new_points_objs = [[points] for points in copy.deepcopy(points_objs_key)]

        for frame_idx in range(self.frame_num):
            gt_boxes_curr = copy.deepcopy(self.gt_boxes_cache[frame_idx])
            if len(gt_boxes_curr) == 0:
                continue
            points_objs_curr = copy.deepcopy(self.points_objs_cache[frame_idx])
            gt_names_curr = copy.deepcopy(self.gt_names_cache[frame_idx])
            enu_pose_curr = copy.deepcopy(self.hs_enu_pose_cache[frame_idx])
            trans_mat = self.pose_trans(enu_pose_curr, enu_pose_key)
            similarity_mat = np.zeros((len(gt_boxes_key), len(gt_boxes_curr)))

            # calculate distance similarty
            center_points = np.array(gt_boxes_curr)[:, :3]
            center_points = np.c_[center_points, np.ones(len(center_points))].T
            center_points = np.dot(trans_mat, center_points)[:3].T
            center_points = np.expand_dims(center_points, 0)
            distance = np.linalg.norm(center_points_key-center_points, axis=-1)
            is_Ped = gt_names_key == 'Pedestrian'
            distance_similarity = 1 / (distance+1) * is_Ped.astype(np.float) + 1 / (distance+1)**0.5 * (~is_Ped).astype(np.float)

            # calculate class similarty
            gt_names_curr = np.expand_dims(gt_names_curr, 0)
            cls_similarity = (gt_names_key == gt_names_curr).astype(np.float)

            # calculate size similarity
            wlh = np.array(gt_boxes_curr)[:, 3:6]
            size = wlh[:,0] * wlh[:,1] * wlh [:,2]
            size = np.expand_dims(size, 0)
            size_similarity = 1 / (np.abs(size_key-size) / (size_key+0.000001) + 1)

            # calculate points number similarity
            points_num = np.array([len(points) for points in points_objs_curr])
            points_num = np.expand_dims(points_num, 0)
            points_num_similarity = 1 / (np.abs(points_num_key-points_num) / (points_num_key+1) + 1) ** 0.5

            # calculate angle similarity                                                                          
            yaw = np.array(gt_boxes_curr)[:, 6] % np.pi
            yaw = np.expand_dims(yaw, 0)
            angle_similarity = ((1 + np.cos(np.abs(yaw_key-yaw))) / 2) ** 2
            
            similarity_mat = distance_similarity * cls_similarity * size_similarity * points_num_similarity * angle_similarity
            try:
                matches, _ = self.hungarian_match(similarity_mat)
            except:
                set_trace()
            for match in matches:
                index_key, index_curr = match
                rot_angle = -gt_boxes_key[index_key][6] + gt_boxes_curr[index_curr][6]
                R = self.rotz(rot_angle)
                rot_points = np.dot(R, (points_objs_curr[index_curr] - gt_boxes_curr[index_curr][:3]).T).T

                # rot_points = rotate_points_along_z(np.expand_dims(points_objs_curr[index_curr] - gt_boxes_curr[index_curr][:3], 0), np.array([rot_angle]))[0]

                # updated_points = np.concatenate((new_points_objs[index_key], rot_points + gt_boxes_key[index_key][:3]), axis=0)
                # new_points_objs[index_key] = updated_points
                new_points_objs[index_key].append(rot_points + gt_boxes_key[index_key][:3])
        for i in range(len(new_points_objs)):
            if len(new_points_objs[i]) == 1:
                new_points_objs[i] = new_points_objs[i][0]
            else:
                new_points_objs[i] = np.concatenate(new_points_objs[i], axis=0)
            # TODO: iterate from key fram in the middle, calculate trajectory similarity
            # a_ = np.random.rand(2,4)
            # b_ = np.random.rand(3,4)
            # c_ = self.iou_batch(a_, b_)
        return new_points_objs

    def hungarian_match(self, similarity_matrix):
        if min(similarity_matrix.shape) > 0:
            a = (similarity_matrix > self.similarity_threshold).astype(np.int32)
            if a.sum(1).max() == 1 and a.sum(0).max() == 1:
                matched_indices = np.stack(np.where(a), axis=1) # ndarray
            else:
                # print(similarity_matrix)
                x,y = linear_sum_assignment(-similarity_matrix) # tuple
                matched_indices = np.array(list(zip(x, y)))

        # filter out matched with low IOU or unmatched label
        matches = []
        similarity = []
        for m in matched_indices:
            if (similarity_matrix[m[0], m[1]] > self.similarity_threshold):
                matches.append(m.reshape(1, 2))
                similarity.append(similarity_matrix[m[0], m[1]])
        if len(matches) == 0:
            matches = np.empty((0, 2), dtype=int)
        else:
            matches = np.concatenate(matches, axis=0)
        
        return matches, np.array(similarity)

    def bg_aggregate(self):

        target_enu_pose = copy.deepcopy(self.hs_enu_pose_cache[self.key_frame_idx])
        target_points_bg_list = [copy.deepcopy(self.points_bg_key_cache[self.key_frame_idx])]
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(self.points_remain_by_range(target_points_bg_list[0], [val*4 for val in pc_range]))
        target_pcd = o3d.geometry.PointCloud.voxel_down_sample(target_pcd, 0.1)
        
        for frame_idx in range(self.frame_num):
            t0 = time.time()
            if frame_idx == self.key_frame_idx or self.hs_enu_pose_cache[frame_idx] is None:
                continue
            source_enu_pose = copy.deepcopy(self.hs_enu_pose_cache[frame_idx])
            trans_mat = self.pose_trans(source_enu_pose, target_enu_pose)
            source_points_bg = copy.deepcopy(self.points_bg_cache[frame_idx])
            source_pcd = o3d.geometry.PointCloud()
            source_pcd.points = o3d.utility.Vector3dVector(self.points_remain_by_range(source_points_bg, [val*4 for val in pc_range]))
            source_pcd = o3d.geometry.PointCloud.voxel_down_sample(source_pcd, 0.1)
            reg_p2p = o3d.registration.registration_icp(
                source_pcd, target_pcd, abs(self.key_frame_idx-frame_idx)/5, trans_mat,
                o3d.registration.TransformationEstimationPointToPoint(),
                o3d.registration.ICPConvergenceCriteria(max_iteration=1000))
            trans_mat = reg_p2p.transformation
            t1 = time.time()
            source_points_bg = np.c_[source_points_bg, np.ones(len(source_points_bg))].T
            t2 = time.time()
            source_points_bg = np.dot(trans_mat, source_points_bg)[:3].T
            t3 = time.time()
            target_points_bg_list.append(source_points_bg)
            t4 = time.time()
        target_points_bg = np.concatenate(target_points_bg_list, axis = 0)
        return target_points_bg

    def points_remove_by_range(self, points, pc_range = [-1.5, 1.5, -1.5, 1.5, -0.8, 0.5]):
        '''
        pc_range: [xmin,xmax,ymin,ymax,zmin,zmax]
        '''
        x_filt = np.logical_and((points[:,0] > pc_range[0]), (points[:,0] < pc_range[1]))
        y_filt = np.logical_and((points[:,1] > pc_range[2]), (points[:,1] < pc_range[3]))
        z_filt = np.logical_and((points[:,2] > pc_range[4]), (points[:,2] < pc_range[5]))
        filter = np.logical_and(np.logical_and(x_filt, y_filt), z_filt)
        indices = np.argwhere(~filter).flatten()
        filter_points = points[indices]

        return filter_points

    def points_remain_by_range(self, points, pc_range = [-1.5, 1.5, -1.5, 1.5, -0.8, 0.5]):
        '''
        pc_range: [xmin,xmax,ymin,ymax,zmin,zmax]
        '''
        x_filt = np.logical_and((points[:,0] > pc_range[0]), (points[:,0] < pc_range[1]))
        y_filt = np.logical_and((points[:,1] > pc_range[2]), (points[:,1] < pc_range[3]))
        z_filt = np.logical_and((points[:,2] > pc_range[4]), (points[:,2] < pc_range[5]))
        filter = np.logical_and(np.logical_and(x_filt, y_filt), z_filt)
        indices = np.argwhere(filter).flatten()
        filter_points = points[indices]

        return filter_points

    def pcd_aggregate(self, points_bg, points_objs):
        t1 = time.time()
        if len(points_objs) != 0:
            points_bg = np.concatenate([points_bg]+points_objs, axis=0)
        t2 = time.time()
        mask = np.random.rand(len(points_bg)) < (4/self.frame_num)
        points = points_bg[mask]
        t3 = time.time()
        return points

    def floor_points_gen(self):
        x = np.arange(self.floor_points_range[0], self.floor_points_range[1], self.voxel_size[0]/4)
        y = np.arange(self.floor_points_range[2], self.floor_points_range[3], self.voxel_size[1]/4)
        ones = np.ones((len(x), len(y)))
        x = np.expand_dims(x, 1)
        y = np.expand_dims(y, 0)
        x = x*ones
        y = y*ones
        z = self.floor_height * ones
        self.floor_points = np.stack([x,y,z], -1).reshape(-1,3)

    def pcd_densify(self, thread_idx=None):
        if thread_idx is not None and get_progress(thread_idx) >= self.frame_num // 2:
            start = get_progress(thread_idx) - self.frame_num // 2
        else:
            start = 0
        for progress_idx, info in tqdm(enumerate(self.content["infos"][start:], start=start)):
            t0 = time.time()
            gt_boxes = info["gt_boxes"]
            try:
                gt_boxes_all = info["gt_boxes_all"]
            except:
                gt_boxes_all = info["gt_boxes"]
            # gt_boxes_all = info["gt_boxes_all"]
            gt_names = info["gt_names"]
            hs_enu_pose = info["hs_enu_pose"]
            scene_token = info["scene_token"]
            lidar_path = info["lidar_path"]
            pcd = o3d.io.read_point_cloud(lidar_path.replace(".bin", ".pcd"))
            points = np.array(pcd.points)
            points_bg_key, points_objs = self.object_segmentation(points, gt_boxes, True)
            points_bg, _ = self.object_segmentation(points, gt_boxes_all, False)

            self.lidar_path_cache.pop(0)
            self.points_cache.pop(0)
            self.gt_boxes_cache.pop(0)
            self.gt_names_cache.pop(0)
            self.hs_enu_pose_cache.pop(0)
            self.scene_token_cache.pop(0)
            self.points_bg_cache.pop(0)
            self.points_bg_key_cache.pop(0)
            self.points_objs_cache.pop(0)

            self.lidar_path_cache.append(lidar_path)
            self.points_cache.append(points)
            self.gt_boxes_cache.append(gt_boxes)
            self.gt_names_cache.append(gt_names)
            self.hs_enu_pose_cache.append(hs_enu_pose)
            self.scene_token_cache.append(scene_token)
            self.points_bg_cache.append(points_bg)
            self.points_bg_key_cache.append(points_bg_key)
            self.points_objs_cache.append(points_objs)
            if self.scene_token_cache[self.key_frame_idx] is None or len(set(self.scene_token_cache)) != 1:
                continue
            if np.linalg.norm(self.hs_enu_pose_cache[-1][:,3] - self.hs_enu_pose_cache[self.key_frame_idx][:,3]) < 4:
                continue

            path_list = ["/"] + self.lidar_path_cache[self.key_frame_idx].split("/")
            pcd_name = path_list[-1].replace(".bin", ".pcd")
            pcd_file_path = path_list[:-2] + ["densification_pcd"]
            pcd_file_path = os.path.join(*pcd_file_path)
            try:
                os.makedirs(pcd_file_path)
            except:
                pass

            aggregated_points_bg = self.bg_aggregate()
            new_points_objs = self.obj_aggregate()
            points = self.pcd_aggregate(aggregated_points_bg, new_points_objs)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(os.path.join(pcd_file_path, pcd_name), pcd)
            save_progress(thread_idx, progress_idx)
            
    def lidar_coor_space_gen(self):
        '''
        return: fake_pc [bev_l, bev_w, bev_h, 3]
        '''
        coor_x = torch.arange(*self.pc_range[:2], self.voxel_size[0])
        coor_y = torch.arange(*self.pc_range[2:4], self.voxel_size[1])
        coor_z = torch.arange(*self.pc_range[4:], self.voxel_size[2])

        bev_l = int((self.pc_range[1] - self.pc_range[0]) / self.voxel_size[0])
        bev_w = int((self.pc_range[3] - self.pc_range[2]) / self.voxel_size[1])
        bev_h = int((self.pc_range[5] - self.pc_range[4]) / self.voxel_size[2])

        coor_x = coor_x.view(bev_l, 1, 1).expand(bev_l, bev_w, bev_h)
        coor_y = coor_y.view(1, bev_w, 1).expand(bev_l, bev_w, bev_h)
        coor_z = coor_z.view(1, 1, bev_h).expand(bev_l, bev_w, bev_h)
        self.fake_pc = torch.stack((coor_x, coor_y, coor_z), -1) + \
            torch.Tensor([self.voxel_size[0], self.voxel_size[1], self.voxel_size[2]]).unsqueeze(0) / 2 #[64,64,20,3]

    def points2img(self, points_lidar, cam_dist, lidar2cam, cam2img):
        try:
            points_cam = points_lidar.tensor[:, :3].matmul(
                lidar2cam[:3, :3].T) + lidar2cam[:3, 3].unsqueeze(0)
        except:
            points_cam = points_lidar[:, :3].matmul(
                lidar2cam[:3, :3].T) + lidar2cam[:3, 3].unsqueeze(0)
        points_dist = torch.norm(points_cam, dim = 1)

        invpol = cam_dist[1]
        
        norm = (points_cam[:, 0] ** 2 + points_cam[:, 1] ** 2) ** 0.5
        theta = np.arctan(points_cam[:, 2] / norm)
        rho = 0
        for i in range(len(invpol)):
            rho += invpol[i] * theta ** i
        x = points_cam[:, 0] / norm * rho
        y = points_cam[:, 1] / norm * rho
        points_cam[:, 0] = x
        points_cam[:, 1] = y
        depth = copy.deepcopy(points_cam[:, 2])
        points_cam[:, 2] = 1
        points_img = points_cam[:, :3].matmul(
            cam2img.float()[:3, :3].T) + cam2img.float()[:3, 3].unsqueeze(0)
        return points_img, points_dist, depth

    def vis_mask_gen(self, points, height, width):
        coor = torch.round(points[:, :2] / 8)
        height, width = int(height / 8), int(width / 8)
        depth = points[:, 2]
        kept1 = (coor[:, 0] >= 0) & (coor[:, 0] < width) & (
            coor[:, 1] >= 0) & (coor[:, 1] < height) & (depth > 0)
        coor, depth = coor[kept1], depth[kept1]
        ranks = coor[:, 0] + coor[:, 1] * width
        sort = (ranks + depth / 100.).argsort()
        coor, depth, ranks = coor[sort], depth[sort], ranks[sort]

        kept2 = torch.ones(coor.shape[0], device=coor.device, dtype=torch.bool)
        kept2[1:] = (ranks[1:] != ranks[:-1])
        return kept1, sort, kept2

    def vis_masks_gen(self, points, info):
        points = torch.from_numpy(points).float()
        cams = ['CAM_FRONT_OV', 'CAM_LEFT_OV', 'CAM_RIGHT_OV', 'CAM_BACK_OV']
        new_points = []
        vis_masks = []
        for cam in cams:

            hs2ego = info['hs2ego']
            cam2ego = np.array(info['cams'][cam]['cam2ego'])
            cam2ego = torch.from_numpy(cam2ego).float()

            hs2ego = torch.from_numpy(hs2ego).float()
            cam2img = np.eye(4, dtype=np.float32)
            cam2img = torch.from_numpy(cam2img)
            cam2img[:3, :3] = torch.from_numpy(np.array(info['cams'][cam]['cam_intrinsic'])).float()

            ego2img = cam2img.float().matmul(torch.inverse(cam2ego))
            lidar2img = ego2img.matmul(hs2ego)
            lidar2cam = torch.inverse(cam2ego).matmul(hs2ego)

            hs2livox = np.array(info['hs2livox'])
            hs2livox = torch.from_numpy(hs2livox).float()
            livox2cam = lidar2cam.matmul(torch.inverse(hs2livox))
    
            tmp = copy.deepcopy(cam2img[0][0:2])
            cam2img[0][0:2] = cam2img[1][0:2]
            cam2img[1][0:2] = tmp
            cam_dist = np.array(info['cams'][cam]['cam_dist'])

            points_img, _, depth = self.points2img(points, cam_dist, livox2cam, cam2img)
            points_img[:, 2] = -depth
            
            img_height = info['cams'][cam]['img_height']
            img_width = info['cams'][cam]['img_width']
            kept1, sort, kept2 = self.vis_mask_gen(points_img, img_height, img_width)
            points_cam = points[kept1][sort]
            new_points.append(points_cam)
            vis_masks.append(kept2)
        new_points = torch.cat(new_points, dim = 0)
        vis_masks = torch.cat(vis_masks, dim = 0)
        return new_points, vis_masks

    def points2depthmap(self, points, height, width):
        coor = torch.round(points[:, :2])
        depth = points[:, 2]
        kept1 = (coor[:, 0] >= 0) & (coor[:, 0] < width) & (
            coor[:, 1] >= 0) & (coor[:, 1] < height) & (
                depth > 0)
        coor, depth = coor[kept1], depth[kept1]
        ranks = coor[:, 0] + coor[:, 1] * width
        sort = (ranks + depth / 100.).argsort()
        coor, depth, ranks = coor[sort], depth[sort], ranks[sort]

        kept2 = torch.ones(coor.shape[0], device=coor.device, dtype=torch.bool)
        kept2[1:] = (ranks[1:] != ranks[:-1])
        coor, depth = coor[kept2], depth[kept2]
        coor = coor.to(torch.long)
        return coor, depth

    def camera_unknown_points_gen(self, occupied_label, info):
        points = copy.deepcopy(self.fake_pc).view(-1,3)
        mask = copy.deepcopy(occupied_label).view(-1)

        occupied_points = points[mask.bool()]
        free_points = points[~mask.bool()]
        cams = ['CAM_FRONT_OV', 'CAM_LEFT_OV', 'CAM_RIGHT_OV', 'CAM_BACK_OV']
        visible_free_indice = []
        out_range_free_kept = torch.ones(len(free_points)).bool()
        # unknown_points = []
        for cam in cams:
            free_indice = torch.range(0, len(free_points)-1)

            hs2ego = info['hs2ego']
            cam2ego = np.array(info['cams'][cam]['cam2ego'])
            cam2ego = torch.from_numpy(cam2ego).float()

            hs2ego = torch.from_numpy(hs2ego).float()
            cam2img = np.eye(4, dtype=np.float32)
            cam2img = torch.from_numpy(cam2img)
            cam2img[:3, :3] = torch.from_numpy(np.array(info['cams'][cam]['cam_intrinsic'])).float()

            ego2img = cam2img.float().matmul(torch.inverse(cam2ego))
            lidar2img = ego2img.matmul(hs2ego)
            lidar2cam = torch.inverse(cam2ego).matmul(hs2ego)

            hs2livox = np.array(info['hs2livox'])
            hs2livox = torch.from_numpy(hs2livox).float()
            livox2cam = lidar2cam.matmul(torch.inverse(hs2livox))
    
            tmp = copy.deepcopy(cam2img[0][0:2])
            cam2img[0][0:2] = cam2img[1][0:2]
            cam2img[1][0:2] = tmp
            cam_dist = np.array(info['cams'][cam]['cam_dist'])
            img_height = int(info['cams'][cam]['img_height'] / self.downsample)
            img_width = int(info['cams'][cam]['img_width'] / self.downsample)
            depth_map_occupied = torch.zeros((img_height, img_width), dtype=torch.float32)
            points_img, _, depth = self.points2img(occupied_points, cam_dist, livox2cam, cam2img)
            points_img[:, 2] = -depth
            points_img[:,:2] = points_img[:,:2] / self.downsample
            coor_occupied, depth_occupied = self.points2depthmap(points_img, img_height, img_width)
            depth_map_occupied[coor_occupied[:, 1], coor_occupied[:, 0]] = depth_occupied

            points_img, _, depth = self.points2img(free_points, cam_dist, livox2cam, cam2img)
            points_img[:,:2] = points_img[:,:2] / self.downsample
            coor_free = torch.round(points_img[:, :2])
            depth_free = -depth

            kept1 = (coor_free[:, 0] >= 0) & (coor_free[:, 0] < img_width) & (
                coor_free[:, 1] >= 0) & (coor_free[:, 1] < img_height) & (
                    depth_free > 0)
            out_range_free_kept = out_range_free_kept & (~kept1)

            coor_free, depth_free, points_img = coor_free[kept1], depth_free[kept1], free_points[kept1]
            coor_free = coor_free.long()
            depth_occupied = depth_map_occupied[coor_free[:, 1], coor_free[:, 0]]
            kept2 = (depth_occupied == 0) | (depth_free < depth_occupied)
            visible_free_indice.append(free_indice[kept1][kept2])
            # unknown_points.append(free_points[kept1][(depth_occupied != 0)&(depth_free > depth_occupied)])
        visible_free_indice = torch.cat(visible_free_indice, dim = 0).long()
        kept = torch.ones(len(free_points)).bool()
        kept[torch.LongTensor(list(set(visible_free_indice.tolist())))] = False
        kept = kept & (~out_range_free_kept)
        unknown_points = free_points[kept]
        # unknown_points = torch.cat(unknown_points, dim=0)
        return unknown_points

    def GetRTMatrix(self, v0, v1):
        '''
        Calculate rotation matrix using v1 v2 from two z-axis
        '''
        v0 = v0 / np.linalg.norm(v0)
        v1 = v1 / np.linalg.norm(v1)
        v2 = np.array([0., 0., 0.])
        v2[0] = v0[1] * v1[2] - v0[2] * v1[1]
        v2[1] = v0[2] * v1[0]- v0[0] * v1[2]
        v2[2] = v0[0] * v1[1]- v0[1] * v1[0]

        RTM = np.array([0. for i in range(9)])
        cosAng = v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2]
        sinAng = math.sqrt(1 - cosAng * cosAng)

        RTM[0] = v2[0] *v2[0] * (1 - cosAng) + cosAng
        RTM[4] = v2[1] *v2[1] * (1 - cosAng) + cosAng
        RTM[8] = v2[2] *v2[2] * (1 - cosAng) + cosAng

        RTM[1] = RTM[3] = v2[0] * v2[1] * (1 - cosAng)
        RTM[2] = RTM[6] = v2[0] * v2[2] * (1 - cosAng)
        RTM[5] = RTM[7] = v2[1] * v2[2] * (1 - cosAng)

        RTM[1] += (v2[2]) * sinAng
        RTM[2] += (-v2[1]) * sinAng
        RTM[3] += (-v2[2]) * sinAng

        RTM[5] += (v2[0]) * sinAng
        RTM[6] += (v2[1]) * sinAng
        RTM[7] += (-v2[0]) * sinAng
        RTM = RTM.reshape(3,3).T
        # return  RTM / np.linalg.norm(RTM, axis = 1) 
        return RTM

    def noise_remove_and_plane_correct(self, pcd, hs2livox):
        # voxel pooling -> transform coor to carcenter -> filter points out of pc_range
        pcd = o3d.geometry.PointCloud.voxel_down_sample(pcd, 0.05)
        points = np.array(pcd.points)
        points = np.c_[points, np.ones(len(points))].T
        points = np.dot(hs2livox, points)[:3].T
        points = self.points_remain_by_range(points, pc_range)
        pcd.points = o3d.utility.Vector3dVector(points)

        # plane segmentation -> remove noise
        plane_model, inliers = pcd.segment_plane(0.1, 3, 2000)
        pcd_plane = pcd.select_down_sample(inliers, invert=False)
        pcd_obstacle = pcd.select_down_sample(inliers, invert=True)
        _, index = pcd_obstacle.remove_radius_outlier(nb_points=12, radius=0.2)
        pcd_obstacle = pcd_obstacle.select_down_sample(index)
        points_obstacle = np.array(pcd_obstacle.points)
        points_plane = np.array(pcd_plane.points)
        points = np.concatenate([np.array(pcd_plane.points), np.array(pcd_obstacle.points)], axis=0)
        RTM = self.GetRTMatrix(np.array(plane_model[:3]), np.array([0,0,1]))
        points = np.dot(RTM, points.T).T + np.array([[0, 0, plane_model[3]-0.149]])
        points_obstacle = np.dot(RTM, points_obstacle.T).T + np.array([[0, 0, plane_model[3]-0.149]])
        points_plane = np.dot(RTM, points_plane.T).T + np.array([[0, 0, plane_model[3]-0.149]])
        if self.pad_floor_points:
            points = np.concatenate([points, self.floor_points], axis = 0)
        points = self.points_remain_by_range(points, pc_range)
        points_obstacle = self.points_remain_by_range(points_obstacle, self.pc_range2)
        return points, points_obstacle

    def occupied_label_gen(self, points, voxel_size):
        max_bound = torch.from_numpy(np.array(pc_range[1::2]))
        min_bound = torch.from_numpy(np.array(pc_range[0::2]))
        intervals = torch.from_numpy(np.array(voxel_size))
        shape = (max_bound - min_bound) / intervals

        occupied_label = torch.zeros(shape.to(torch.int).tolist()).to(torch.int)

        coor = (points-min_bound-0.00001) / torch.from_numpy(np.array(voxel_size))
        coor = coor.to(torch.long)
        ranks = coor[:, 2] * shape[0] * shape[1]  + coor[:, 1] * shape[0] + coor[:,0]
        sort = ranks.argsort()
        ranks = ranks[sort]
        kept = torch.ones(coor.shape[0], device=coor.device, dtype=torch.bool)
        kept[1:] = (ranks[1:] != ranks[:-1])
        coor = coor[kept]
        occupied_label[coor[:, 1], coor[:, 0], coor[:, 2]] = 1
        mask_lidar_fake = np.zeros(shape.to(torch.int).tolist(), dtype=np.uint8)
        mask_camera_fake = np.zeros(shape.to(torch.int).tolist(), dtype=np.uint8)
        return occupied_label, mask_lidar_fake, mask_camera_fake

    def up_samlple_mask_gen(self):
        h = int((self.pc_range2[1] - self.pc_range2[0]) / self.voxel_size2[0])
        w = int((self.pc_range2[3] - self.pc_range2[2]) / self.voxel_size2[1])
        self.mask = np.ones((h, w))
        start_h = int((h - self.up_sample_mask[0]) / 2)
        start_w = int((w - self.up_sample_mask[1]) / 2)
        self.mask[start_h:start_h+self.up_sample_mask[0], start_w:start_w+self.up_sample_mask[1]] = 0
        self.mask = torch.Tensor(self.mask).bool()

    def upsample_label(self, label):
        label = label.permute(2,0,1)
        label_1 = label * (~self.mask)
        label2 = F.max_pool2d(label.float(), 3, 1, 1)
        label_2 = label2 * self.mask
        label = label_1 + label_2
        return label.permute(1,2,0).int()


    def occ_label_gen(self):
        """
        Generate occupancy ground-truth for each frame (offline).
        The output is a voxel grid with:
            0: free
            1: occupied
            2: unknown (not visible by camera)
        """

        # The lidar path must come from hs64 sensor
        assert "hs64" in self.content["infos"][0]['lidar_path']

        # Create output directory by replacing hs64 -> occ
        occ_file_path = os.path.split(
            self.content["infos"][0]['lidar_path']
        )[0].replace("hs64", "occ")

        assert "occ" in occ_file_path
        if not os.path.exists(occ_file_path):
            os.makedirs(occ_file_path)
            
        # Iterate over all frames (offline infos)
        for info in tqdm(self.content["infos"]):

            # Convert lidar bin path to densified pcd path
            path_list = ["/"] + info['lidar_path'].split("/")
            pcd_name = path_list[-1].replace(".bin", ".pcd")

            # Densified point cloud directory
            pcd_file_path = path_list[:-2] + ["densification_pcd"]
            pcd_file_path = os.path.join(*pcd_file_path)

            # Load densified point cloud
            if os.path.exists(os.path.join(pcd_file_path, pcd_name)):
                pcd = o3d.io.read_point_cloud(
                    os.path.join(pcd_file_path, pcd_name)
                )
            else:
                continue

            # Remove noise and correct ground plane
            # points: non-obstacle
            # points_obstacle: obstacle points
            points, points_obstacle = self.noise_remove_and_plane_correct(
                pcd, info['hs2livox']
            )

            # Generate camera visibility masks
            new_points, vis_masks = self.vis_masks_gen(points, info)
            new_points_obstacle, vis_masks_obstacle = \
                self.vis_masks_gen(points_obstacle, info)

            # Keep only camera-visible points
            vis_points = new_points[vis_masks]
            vis_points_obstacle = new_points_obstacle[vis_masks_obstacle]

            # Generate occupied voxels from visible points
            occupied_label, mask_lidar_fake, mask_camera_fake = \
                self.occupied_label_gen(vis_points, voxel_size)

            # Generate unknown region (camera invisible voxels)
            unknown_points = self.camera_unknown_points_gen(
                occupied_label, info
            )
            unknown_label, _, _ = self.occupied_label_gen(
                unknown_points, voxel_size
            )

            # Merge occupied (1) and unknown (2)
            occupied_label += unknown_label * 2

            # Save occupancy ground truth
            save_path_ = occ_file_path + "/" + pcd_name.replace(".pcd", ".npz")
            info["occ_gt_path"] = save_path_

            np.savez_compressed(
                save_path_,
                semantics=np.array(occupied_label),
                mask_lidar=mask_lidar_fake,
                mask_camera=mask_camera_fake
            )


    def voxel2points(self, voxel, voxelSize=[0.4, 0.4, 0.4], pc_range=[-40.0, -40.0, -1.0, 40.0, 40.0, 5.4]):

        points = []
        wl = int((pc_range[3]-pc_range[0])/voxelSize[0])
        h = int((pc_range[5]-pc_range[2])/voxelSize[2])
        for i in range(wl):
            for j in range(wl):
                for k in range(h):
                    if voxel[i,j,k] > 0:
                        points.append([i*voxelSize[0]+pc_range[0], j*voxelSize[1]+pc_range[1], k*voxelSize[2]+pc_range[2]])
        return np.array(points)

    def vis(self, processed_label_occ, pc_range, voxelSize):
        pcd_occ = o3d.geometry.PointCloud()
        points_occ = self.voxel2points(processed_label_occ, voxelSize = voxelSize, pc_range = pc_range)
        pcd_occ.points = o3d.utility.Vector3dVector(points_occ)

        o3d.io.write_point_cloud("~/Occ-BEV/occ.pcd", pcd_occ)

def localtime2int_info(info):
    lidar_path = info['lidar_path']
    lidar_name =  lidar_path.split("/")[-1]
    localtime = lidar_name.replace('-','').replace('.pcd','').replace('.txt','').replace('.bin','')
    return int(localtime)

def get_progress(thread_id):
    path = '~/Occ_dataset_gen/thread_progress/thread_'+str(thread_id)+'.txt'
    try:
        with open(path, 'r') as f:
            return int(f.read())
    except FileNotFoundError:
        return 0

def save_progress(thread_id, value):
    with open('~/Occ_dataset_gen/thread_progress/thread_'+str(thread_id)+'.txt', 'w') as f:
        f.write(str(value))

if __name__ == "__main__":
    global similarity_threshold, filter_near, pc_range_filters, pc_range,\
           voxel_size, downsample, pad_floor_points, floor_points_range, floor_height, pc_range2
    pad_floor_points = True
    similarity_threshold = 0.15
    filter_near = True
    pc_range_filters = [[-1.5, 1.5, -1.5, 1.5, -1.345 + 0.5, 2.5], [-25.6, 25.6, -25.6, 25.6, -1.5-1.345, -1.345 - 0.5]]
    pc_range = [-25.6, 25.6, -25.6, 25.6, -5, 3]
    floor_points_range = [-2.5, 2.5, -2.5, 2.5]
    voxel_size = [0.4, 0.4, 0.4]
    downsample = 12
    floor_height = -0.149
    pc_range2 = [-25.6, 25.6, -25.6, 25.6, -5, 3]

    print(sys.argv)
    pkl_path, frame_num, thread_idx, thread_sum = sys.argv[1:5]
    frame_num, thread_idx, thread_sum = int(frame_num), int(thread_idx), int(thread_sum)
    with open(pkl_path, "rb") as f:
        content_sum = pickle.load(f)
    content_sum['infos'].sort(key = localtime2int_info)

    infos_len = len(content_sum['infos'])
    son_infos_len = int(infos_len / int(thread_sum) + 1)
    content_thread = copy.deepcopy(content_sum)
    min_idx = max(0, thread_idx * son_infos_len - int(frame_num/2))
    max_idx = min(infos_len-1, (thread_idx+1) * son_infos_len + int(frame_num/2))
    content_thread['infos'] = content_sum['infos'][min_idx:max_idx]
    del content_sum
    runner = OccDatasetGen(pkl_path, content_thread, int(frame_num))
    runner.pcd_densify(thread_idx)
    # runner.occ_label_gen()
    