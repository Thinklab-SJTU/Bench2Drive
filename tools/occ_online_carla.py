import numpy as np
import torch
import open3d as o3d
from leaderboard.utils.occ_label_gen import OccDatasetGen
from collections import deque, Counter, defaultdict
from itertools import product
import time

# True: dynamic
CARLA_SEMANTIC_LABELS = {
    0: ("unlabeled", (0, 0, 0), False),
    1: ("road", (128, 64, 128), False),
    2: ("sidewalk", (244, 35, 232), False),
    3: ("building", (70, 70, 70), False),
    4: ("wall", (102, 102, 156), False),
    5: ("fence", (190, 153, 153), False),
    6: ("pole", (153, 153, 153), False),
    7: ("traffic_light", (250, 170, 30), False),
    8: ("traffic_sign", (220, 220, 0), False),
    9: ("vegetation", (107, 142, 35), False),
    10: ("terrain", (152, 251, 152), False),
    11: ("sky", (70, 130, 180), False),
    12: ("pedestrian", (220, 20, 60), True),
    13: ("rider", (255, 0, 0), True),
    14: ("car", (0, 0, 142), True),
    15: ("truck", (0, 0, 70), True),
    16: ("bus", (0, 60, 100), True),
    17: ("train", (0, 80, 100), True),
    18: ("motorcycle", (0, 0, 230), True),
    19: ("bicycle", (119, 11, 32), True),
    20: ("static", (110, 190, 160), False),
    21: ("dynamic", (170, 120, 50), True),
    22: ("other", (55, 90, 80), False),
    23: ("water", (45, 60, 150), False),
    24: ("road_line", (157, 234, 50), False),
    25: ("ground", (81, 0, 81), False),
    26: ("bridge", (150, 100, 100), False),
    27: ("rail_track", (230, 150, 140), False),
    28: ("guard_rail", (180, 165, 180), False),
}

FILLED_DYNAMIC_LABELS = [
    14, # car
    15, # bus
    16, # bus
    17, # train
]

FILLED_STATIC_LABELS = [
    1, # road
    2, # sidewalk
    3, # building
    4, # wall
    10, # terrain
    23, # water
    24, # road_line
    25, # ground
]

class OccOnlineCARLA:
    """
    Online occupancy generator for CARLA.
    LiDAR-only, ego-centric, tick-level.
    """

    def __init__(self, history_length=10):
        self.pc_range = [-25.6, 25.6, -25.6, 25.6, -5, 3]
        self.voxel_size = [0.4, 0.4, 0.4]

        # frame cache
        self.frame_cache = None
        self.history_length = history_length
        self.frame_cache_queue = deque(maxlen=history_length)
        self.static_cache = []  # list
        self.static_step = 0.4
        self.static_length = 16
        self.ground_flood_threshold = 128
        self.hollow_stride = 8
    
    def clean_cache(self):
        self.frame_cache = None
        self.frame_cache_queue = deque(maxlen=self.history_length)
        self.static_cache = []

    @staticmethod
    def world_to_ego(points_world, world2ego):
        """
        Transform points from world to ego frame using 4x4 matrix.
        """
        N = points_world.shape[0]
        homo = np.hstack([points_world, np.ones((N,1), dtype=np.float32)])
        points_ego = (world2ego @ homo.T).T[:, :3]
        return points_ego
    
    def parse_single_semantic_lidar(self, lidar):
        """
        Parse semantic lidar into xyz, obj_tag, obj_id arrays
        """
        if isinstance(lidar, tuple):
            raw = lidar[1]
        elif isinstance(lidar, np.ndarray):
            raw = lidar
        else:
            raw = np.frombuffer(lidar.raw_data, dtype=np.float32).reshape(-1, 6)

        raw = raw.astype(np.float32, copy=False)
        xyz = raw[:, :3]
        obj_tag = raw[:, 5].view(np.uint32)
        obj_id = raw[:, 4].view(np.int32)

        # if obj_tag.size > 0:
        #     print(f"[debug] raw lidar: pts={xyz.shape[0]}, sem min={obj_tag.min()} max={obj_tag.max()}, instance min={obj_id.min()} max={obj_id.max()}")
        # else:
        #     print("[debug] raw lidar: empty")

        return {"xyz": xyz, "obj_tag": obj_tag, "obj_id": obj_id}
    
    def build_world2obj_from_bbox(self, b):
        # 1. fuzzy match: any key starting with "world2"
        for k, v in b.items():
            if k.startswith("world2"):
                return np.array(v, dtype=np.float32)

        # 2. fallback: build from location + rotation
        location = np.array(b["location"], dtype=np.float32)
        rot = b["rotation"]
        roll, pitch, yaw = np.deg2rad(rot)

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)],
        ])
        Ry = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1],
        ])

        R = Rz @ Ry @ Rx

        obj2world = np.eye(4, dtype=np.float32)
        obj2world[:3, :3] = R
        obj2world[:3, 3] = location

        return np.linalg.inv(obj2world)

    def _print_range(self, name, arr):
        if arr is None or len(arr) == 0:
            print(f"[debug] {name}: EMPTY")
            return
        mn = arr.min(axis=0)
        mx = arr.max(axis=0)
        print(f"[debug] {name}: "
            f"x[{mn[0]:.2f}, {mx[0]:.2f}] "
            f"y[{mn[1]:.2f}, {mx[1]:.2f}] "
            f"z[{mn[2]:.2f}, {mx[2]:.2f}] "
            f"pts={arr.shape[0]}")
    
    def bresenham3D(self, start, end):
        """
        3D Bresenham line algorithm
        start, end: tuple(int,int,int) in (y,x,z) or whatever convention you use
        returns: list of tuple(int,int,int)
        """
        x0, y0, z0 = start
        x1, y1, z1 = end

        voxels = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        dz = abs(z1 - z0)

        xs = 1 if x1 > x0 else -1
        ys = 1 if y1 > y0 else -1
        zs = 1 if z1 > z0 else -1

        if dx >= dy and dx >= dz:
            py = 2 * dy - dx
            pz = 2 * dz - dx
            while x0 != x1:
                voxels.append((x0, y0, z0))
                x0 += xs
                if py >= 0:
                    y0 += ys
                    py -= 2 * dx
                if pz >= 0:
                    z0 += zs
                    pz -= 2 * dx
                py += 2 * dy
                pz += 2 * dz

        elif dy >= dx and dy >= dz:
            px = 2 * dx - dy
            pz = 2 * dz - dy
            while y0 != y1:
                voxels.append((x0, y0, z0))
                y0 += ys
                if px >= 0:
                    x0 += xs
                    px -= 2 * dy
                if pz >= 0:
                    z0 += zs
                    pz -= 2 * dy
                px += 2 * dx
                pz += 2 * dz

        else:
            px = 2 * dx - dz
            py = 2 * dy - dz
            while z0 != z1:
                voxels.append((x0, y0, z0))
                z0 += zs
                if px >= 0:
                    x0 += xs
                    px -= 2 * dz
                if py >= 0:
                    y0 += ys
                    py -= 2 * dz
                px += 2 * dx
                py += 2 * dy

        voxels.append((x1, y1, z1))
        return voxels

    
    def fill_dynamic_object_voxel(
        self,
        occ, sem,
        voxel_size, pc_range,
        world2ego,
        obj_center, obj_extent, obj_yaw,
        obj_label,
    ):
        """
        Axis-safe, type-safe OBB voxel filling with supersampling + voting + ray fill.
        Voxel axis convention: (y, x, z)
        """
        
        if not obj_label in FILLED_DYNAMIC_LABELS:
            # print("[debug] This object will not be filled.")
            return occ, sem

        device = occ.device
        voxel_size_t = torch.tensor(voxel_size, device=device, dtype=torch.float32)
        min_bound_t = torch.tensor(pc_range[0::2], device=device, dtype=torch.float32)

        # ----------------------------
        # helpers
        # ----------------------------
        world2ego_t = torch.from_numpy(world2ego).to(device=device, dtype=torch.float32)

        def world_to_ego(pts_world: torch.Tensor) -> torch.Tensor:
            """(N,3) world -> (N,3) ego"""
            ones = torch.ones((pts_world.shape[0], 1), device=device)
            pts_h = torch.cat([pts_world, ones], dim=1)  # (N,4)
            return (world2ego_t @ pts_h.T).T[:, :3]

        def ego_to_voxel_yxz(pts_ego: torch.Tensor) -> torch.Tensor:
            """
            ego xyz -> voxel (y,x,z)
            """
            v_xyz = torch.floor((pts_ego - min_bound_t) / voxel_size_t).long()
            return v_xyz[:, [1, 0, 2]]  # y,x,z

        def filter_valid(v: torch.Tensor) -> torch.Tensor:
            mask = (
                (v[:, 0] >= 0) & (v[:, 0] < occ.shape[0]) &
                (v[:, 1] >= 0) & (v[:, 1] < occ.shape[1]) &
                (v[:, 2] >= 0) & (v[:, 2] < occ.shape[2])
            )
            return v[mask]

        # ----------------------------
        # OBB parameters
        # ----------------------------
        hx, hy, hz = obj_extent
        yaw = np.deg2rad(obj_yaw)

        c, s = np.cos(yaw), np.sin(yaw)
        R = torch.tensor(
            [[c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0]],
            device=device, dtype=torch.float32
        )

        obj_center_t = torch.tensor(obj_center, device=device, dtype=torch.float32)

        # ----------------------------
        # compute AABB in ego voxel space
        # ----------------------------
        corners_local = torch.tensor(
            list(product([-hx, hx], [-hy, hy], [-hz, hz])),
            device=device, dtype=torch.float32
        )

        corners_world = corners_local @ R.T + obj_center_t
        corners_ego = world_to_ego(corners_world)
        corners_vox = ego_to_voxel_yxz(corners_ego)

        corners_vox = filter_valid(corners_vox)

        if corners_vox.numel() == 0:
            return occ, sem

        min_y, min_x, min_z = corners_vox.min(dim=0).values.tolist()
        max_y, max_x, max_z = corners_vox.max(dim=0).values.tolist()

        min_y, min_x, min_z = int(min_y), int(min_x), int(min_z)
        max_y, max_x, max_z = int(max_y), int(max_x), int(max_z)

        # ----------------------------
        # supersampling (half voxel)
        # ----------------------------
        step = voxel_size_t * 0.5

        # inside samples (B)
        xs = torch.arange(-hx + step[0]/2, hx, step[0], device=device)
        ys = torch.arange(-hy + step[1]/2, hy, step[1], device=device)
        zs = torch.arange(-hz + step[2]/2, hz, step[2], device=device)

        Xin, Yin, Zin = torch.meshgrid(xs, ys, zs, indexing="ij")
        pts_B_local = torch.stack([Xin.flatten(), Yin.flatten(), Zin.flatten()], dim=1)

        # outside shell samples (A)
        pad = voxel_size_t
        xs = torch.arange(-hx - pad[0] + step[0]/2, hx + pad[0], step[0], device=device)
        ys = torch.arange(-hy - pad[1] + step[1]/2, hy + pad[1], step[1], device=device)
        zs = torch.arange(-hz - pad[2] + step[2]/2, hz + pad[2], step[2], device=device)

        Xo, Yo, Zo = torch.meshgrid(xs, ys, zs, indexing="ij")
        pts_A_local = torch.stack([Xo.flatten(), Yo.flatten(), Zo.flatten()], dim=1)

        mask_out = ~(
            (pts_A_local[:, 0].abs() <= hx) &
            (pts_A_local[:, 1].abs() <= hy) &
            (pts_A_local[:, 2].abs() <= hz)
        )
        pts_A_local = pts_A_local[mask_out]

        # local -> world -> ego
        pts_B_ego = world_to_ego(pts_B_local @ R.T + obj_center_t)
        pts_A_ego = world_to_ego(pts_A_local @ R.T + obj_center_t)

        vox_B = filter_valid(ego_to_voxel_yxz(pts_B_ego))
        vox_A = filter_valid(ego_to_voxel_yxz(pts_A_ego))
        
        # print(f"[debug] pts_B_local = {pts_B_local}")
        # print(f"[debug] pts_A_local = {pts_A_local}")
        # print(f"[debug] vox_B(valid) = {vox_B}")
        # print(f"[debug] vox_A(valid) = {vox_A}")

        # ----------------------------
        # voting
        # ----------------------------
        vote_B = defaultdict(int)
        vote_A = defaultdict(int)

        for v in vox_B.tolist():
            vote_B[tuple(v)] += 1
        for v in vox_A.tolist():
            vote_A[tuple(v)] += 1

        voxels_in_bbox = []
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                for z in range(min_z, max_z + 1):
                    if vote_B.get((y, x, z), 0) > vote_A.get((y, x, z), 0):
                        voxels_in_bbox.append((y, x, z))

        voxels_in_bbox_set = set(voxels_in_bbox)

        # ----------------------------
        # ray-based filling
        # ----------------------------
        # ----------------------------
        # stride thick ray filling
        # ----------------------------
        original_occ = occ.clone()
        H, W, D = occ.shape

        def in_bounds(v):
            y, x, z = v
            return 0 <= y < H and 0 <= x < W and 0 <= z < D

        center_ego = world_to_ego(obj_center_t[None])[0]
        center_vox = ego_to_voxel_yxz(center_ego[None])[0]
        center_vox = tuple(int(v) for v in center_vox)

        # contruct surface voxel (AABB faces)
        surface_voxels = []
        for y in [min_y, max_y]:
            for x in range(min_x, max_x + 1):
                for z in range(min_z, max_z + 1):
                    surface_voxels.append((y, x, z))
        for x in [min_x, max_x]:
            for y in range(min_y, max_y + 1):
                for z in range(min_z, max_z + 1):
                    surface_voxels.append((y, x, z))
        for z in [min_z, max_z]:
            for y in range(min_y, max_y + 1):
                for x in range(min_x, max_x + 1):
                    surface_voxels.append((y, x, z))
        surface_voxels = list(set(surface_voxels))  # delete repetition

        # print(f"[debug] surface_voxels_len = {len(surface_voxels)}")

        # ----------------------------
        # fill with thic ray
        # ----------------------------
        stride = 2  # adjustable
        visited = set()
        filled_by_ray = 0
        hit_occupied_rays = 0

        # use stride to split surface voxel
        blocks = []
        for sv in surface_voxels:
            # every block is a small section near stride*stride
            y0, x0, z0 = sv
            block = [(y0+dy, x0+dx, z0+dz)
                    for dy in range(stride)
                    for dx in range(stride)
                    for dz in range(stride)
                    if y0+dy <= max_y and x0+dx <= max_x and z0+dz <= max_z]
            blocks.append(block)

       # use stride to split surface voxel
        for block in blocks:
            # acquire all voxels from all rays in the block
            all_voxels = []
            for sv in block:
                ray = self.bresenham3D(center_vox, sv)
                all_voxels.extend(ray)
            
            # delete repition and rank by distances to the center
            all_voxels = list(set(all_voxels))
            all_voxels.sort(key=lambda v: (v[0]-center_vox[0])**2 + (v[1]-center_vox[1])**2 + (v[2]-center_vox[2])**2)

            # iterate all voxels from the center to the surface
            for v in all_voxels:
                if not in_bounds(v):
                    continue
                if v not in voxels_in_bbox_set:
                    continue
                if tuple(v) in visited:
                    continue

                visited.add(tuple(v))

                # hit originally occupied → successing voxels in the block are all free, stop filling
                if original_occ[v[0], v[1], v[2]] == 1:
                    hit_occupied_rays += 1
                    break

                # fill occupancy
                occ[v[0], v[1], v[2]] = 1
                sem[v[0], v[1], v[2]] = obj_label
                filled_by_ray += 1
        
        # print(f"[debug] filled_by_ray = {filled_by_ray}")
        # print(f"[debug] hit_occupied_rays = {hit_occupied_rays}")
        # print(f"[debug] visited_voxels = {len(visited)}")

        return occ, sem
    
    
    def hollow_fix(self, occ, sem, obj_id_grid, stride=4):
        """
        occ: (Y, X, Z) uint8
        sem: (Y, X, Z) int16
        obj_id_grid: (Y, X, Z) int32
        stride: coarse grid size
        """

        t0 = time.time()
        Y, X, Z = occ.shape
        
        # ------------------ seperate dynamic voxel, including small static ones ------------------
        dynamic_labels = [k for k, v in CARLA_SEMANTIC_LABELS.items() if k not in FILLED_STATIC_LABELS]  # True represents dynamic
        dynamic_labels_t = torch.tensor(dynamic_labels, device=sem.device)
        dynamic_mask = torch.isin(sem, dynamic_labels_t) & (occ == 1)

        # save dynamic voxels
        occ_dynamic = occ.clone()
        sem_dynamic = sem.clone()
        obj_id_dynamic = obj_id_grid.clone()

        # erase dynamic voxels from occ/sem,keep static only
        occ[dynamic_mask] = 0
        sem[dynamic_mask] = -1
        obj_id_grid[dynamic_mask] = -1

        # ---------------- Step 0: precompute top_occ ----------------
        # top_occ[y,x] = highest z where occ[y,x,z]==1, -1 if none
        top_occ = np.full((Y, X), -1, dtype=np.int32)
        for z in range(0, Z):
            mask = (occ[:, :, z] == 1)
            top_occ[mask] = z
        # print(f"[debug][hollow] step0 top_occ precomputed: {time.time() - t0:.3f}s")
        # print(f"[debug][hollow] top_occ = {top_occ}")
        # print(f"[debug][hollow] top_occ min={top_occ.min()}, max={top_occ.max()}, mean={top_occ.mean():.2f}")

        # ---------------- Step 1: coarse grid blockify ----------------
        Hc = (Y + stride - 1) // stride
        Wc = (X + stride - 1) // stride
        coarse_type = np.zeros((Hc, Wc), dtype=np.int8)  # 0: no free, 1: partial free, 2: full free

        for cy in range(Hc):
            for cx in range(Wc):
                y0, y1 = cy*stride, min((cy+1)*stride, Y)
                x0, x1 = cx*stride, min((cx+1)*stride, X)
                block_top = top_occ[y0:y1, x0:x1]
                n_voxels = block_top.size
                n_free = np.sum(block_top == -1)
                if n_free == 0:
                    coarse_type[cy, cx] = 0  # all occupied. no need to process
                elif n_free == n_voxels:
                    coarse_type[cy, cx] = 2  # all free
                else:
                    coarse_type[cy, cx] = 1  # partial free,need BFS
        # stats
        n_no_free = np.sum(coarse_type == 0)
        n_partial_free = np.sum(coarse_type == 1)
        n_full_free = np.sum(coarse_type == 2)
        # print(f"[debug][hollow] step1 coarse grid classified: {time.time() - t0:.3f}s")
        # print(f"[debug][hollow] coarse grid counts: no_free={n_no_free}, partial_free={n_partial_free}, full_free={n_full_free}")

        # ---------------- Step 2: deal with partial free ----------------
        filled_count = 0
        visited_xy = np.zeros((Y, X), dtype=bool)

        for cy in range(Hc):
            for cx in range(Wc):
                if coarse_type[cy, cx] != 1:
                    continue

                y0, y1 = cy*stride, min((cy+1)*stride, Y)
                x0, x1 = cx*stride, min((cx+1)*stride, X)

                # do voxel-level BFS within a block
                for sy in range(y0, y1):
                    for sx in range(x0, x1):
                        if top_occ[sy, sx] != -1 or visited_xy[sy, sx]:
                            continue

                        queue = deque([(sy, sx)])
                        region = [(sy, sx)]
                        visited_xy[sy, sx] = True
                        boundary_sem = []
                        boundary_z = []

                        while queue:
                            y, x = queue.popleft()
                            for ny, nx in [(y-1,x),(y+1,x),(y,x-1),(y,x+1)]:
                                if not (y0 <= ny < y1 and x0 <= nx < x1):
                                    continue
                                if visited_xy[ny, nx]:
                                    continue
                                if top_occ[ny, nx] != -1:
                                    # boundary voxel
                                    boundary_sem.append(int(sem[ny, nx, top_occ[ny,nx]]))
                                    boundary_z.append(int(top_occ[ny, nx]))
                                    visited_xy[ny, nx] = True
                                    continue
                                visited_xy[ny, nx] = True
                                queue.append((ny, nx))
                                region.append((ny, nx))

                        if not boundary_sem:
                            continue

                        # fill
                        fill_sem = Counter(boundary_sem).most_common(1)[0][0]
                        fill_z = max(z for z, s in zip(boundary_z, boundary_sem) if s == fill_sem)
                        
                        for y, x in region:
                            if occ[y, x, fill_z] == 0:
                                occ[y, x, fill_z] = 1
                                sem[y, x, fill_z] = fill_sem
                                obj_id_grid[y, x, fill_z] = 0
                                top_occ[y, x] = fill_z        # update top_occ
                                filled_count += 1 

        # print(f"[debug][hollow] step2 half-free filled: {filled_count} voxels, {time.time() - t0:.3f}s")
        
        # ---------------- Step 3: process full-free coarse blocks (grid-level BFS) ----------------
        filled_count_full = 0
        visited_grid = np.zeros((Hc, Wc), dtype=bool)

        for cy in range(Hc):
            for cx in range(Wc):
                if coarse_type[cy, cx] != 2 or visited_grid[cy, cx]:
                    continue

                queue = deque([(cy, cx)])
                region_grids = [(cy, cx)]
                visited_grid[cy, cx] = True

                boundary_sem = []
                boundary_z = []

                while queue:
                    gcy, gcx = queue.popleft()

                    # bfs
                    for ngy, ngx, direction in [
                        (gcy-1, gcx, 'up'),
                        (gcy+1, gcx, 'down'),
                        (gcy, gcx-1, 'left'),
                        (gcy, gcx+1, 'right')
                    ]:

                        if not (0 <= ngy < Hc and 0 <= ngx < Wc):
                            continue
                        if visited_grid[ngy, ngx]:
                            continue

                        y0, y1 = ngy*stride, min((ngy+1)*stride, Y)
                        x0, x1 = ngx*stride, min((ngx+1)*stride, X)

                        if coarse_type[ngy, ngx] == 2:
                            # also full-free grid
                            visited_grid[ngy, ngx] = True
                            queue.append((ngy, ngx))
                            region_grids.append((ngy, ngx))
                        else:
                            # only ones adjacent to the border
                            if direction == 'up':
                                y_indices = y1 - 1
                                x_indices = slice(x0, x1)
                            elif direction == 'down':
                                y_indices = y0
                                x_indices = slice(x0, x1)
                            elif direction == 'left':
                                y_indices = slice(y0, y1)
                                x_indices = x1 - 1
                            elif direction == 'right':
                                y_indices = slice(y0, y1)
                                x_indices = x0
                            # mask method stats
                            if isinstance(y_indices, int):
                                ys = np.array([y_indices])
                            else:
                                ys = np.arange(y_indices.start, y_indices.stop)
                            if isinstance(x_indices, int):
                                xs = np.array([x_indices])
                            else:
                                xs = np.arange(x_indices.start, x_indices.stop)

                            yy, xx = np.meshgrid(ys, xs, indexing='ij')
                            yy = yy.ravel()
                            xx = xx.ravel()
                            top_vals = top_occ[yy, xx]
                            valid = top_vals != -1
                            boundary_sem.extend(sem[yy[valid], xx[valid], top_vals[valid]].tolist())
                            boundary_z.extend(top_vals[valid].tolist())

                if not boundary_sem:
                    continue

                # fill
                fill_sem = Counter(boundary_sem).most_common(1)[0][0]
                fill_z = max(z for z, s in zip(boundary_z, boundary_sem) if s == fill_sem)
                # print(f"[debug] fill_sem = {fill_sem}, fill_z = {fill_z}")

                # batch mask fill region
                for gcy, gcx in region_grids:
                    y0, y1 = gcy*stride, min((gcy+1)*stride, Y)
                    x0, x1 = gcx*stride, min((gcx+1)*stride, X)
                    mask = (occ[y0:y1, x0:x1, fill_z] == 0)
                    sem[y0:y1, x0:x1, fill_z][mask] = fill_sem
                    occ[y0:y1, x0:x1, fill_z][mask] = 1
                    obj_id_grid[y0:y1, x0:x1, fill_z][mask] = 0
                    top_occ[y0:y1, x0:x1][mask] = fill_z
                    filled_count_full += mask.sum()

        # print(f"[debug][hollow] step3 full-free grid BFS filled: {filled_count_full} voxels, {time.time() - t0:.3f}s")

        # print(f"[debug][hollow] hollow fix total time: {time.time() - t0:.3f}s")
        
        # ------------------ assembly dynamic voxel ------------------
        occ[dynamic_mask] = occ_dynamic[dynamic_mask]
        sem[dynamic_mask] = sem_dynamic[dynamic_mask]
        obj_id_grid[dynamic_mask] = obj_id_dynamic[dynamic_mask]

        return occ, sem, obj_id_grid
    
    
    def downward_fill(self, occ, sem, obj_id_grid):
        """
        occ, sem, obj_id_grid: torch.Tensor
        shape: (Y, X, Z)
        """
        t0 = time.time()
        device = occ.device
        Y, X, Z = occ.shape

        # find occ in the bottom
        # argmax (bool -> True=1,
        bottom_z = torch.argmax(occ, dim=2)  # (Y,X)

        Y_idx = torch.arange(Y, device=device)[:, None].expand(Y, X)
        X_idx = torch.arange(X, device=device)[None, :].expand(Y, X)

        bottom_sem = sem[Y_idx, X_idx, bottom_z]      # (Y,X)

        # create filling down mask
        z_grid = torch.arange(Z, device=device)[None, None, :]  # (1,1,Z)
        fill_mask = z_grid < bottom_z[:, :, None]               # (Y,X,Z)

        # 5️fill
        occ[fill_mask] = 1
        sem[fill_mask] = bottom_sem[:, :, None].expand(Y, X, Z)[fill_mask]
        obj_id_grid[fill_mask] = 0

        # print(f"[debug][down-fill] filled {fill_mask.sum().item()} voxels, "
        #     f"time {time.time() - t0:.3f}s")

    def occ_from_carla_tick(
        self,
        lidar_segs,
        bbox,
        sensors_anno,
        lidar_keys=None
    ):
        """
        Occupancy + semantic voxel grid from CARLA semantic LiDAR(s),
        with frame cache.
        """

        # Normalize inputs
        if not isinstance(lidar_segs, (list, tuple)):
            lidar_segs = [lidar_segs]

        N = len(lidar_segs)
        if lidar_keys is None:
            lidar_keys = [f"LIDAR_{i}" for i in range(N)]
        assert len(lidar_keys) == N

        xyz_all, tag_all, id_all, points_world_all = [], [], [], []
        
        ego_bbox = next((b for b in bbox if b.get("class")=="ego_vehicle"), None)
        ego_pose = np.eye(4, dtype=np.float32)
        if ego_bbox is not None:
            ego_pose = np.array(ego_bbox.get("world2ego", np.eye(4)), dtype=np.float32)
            # print(f"[debug] using ego_vehicle world2ego for ego_pose:\n{ego_pose}")

        # Parse LiDAR and transform to ego
        for idx, (lidar, key) in enumerate(zip(lidar_segs, lidar_keys)):
            semantic = self.parse_single_semantic_lidar(lidar)
            if semantic["xyz"].size == 0:
                continue

            # get world2lidar
            # print(f"[debug] sensor_anno's keys: {sensors_anno.keys()}")
            world2sensor = np.array(sensors_anno[key]["world2lidar"], dtype=np.float32)
            points_world = semantic["xyz"]  # semantic lidar is in sensor coords
            # Transform to world
            sensor2world = np.linalg.inv(world2sensor)
            homo = np.hstack([points_world, np.ones((points_world.shape[0],1), dtype=np.float32)])
            points_world = (sensor2world @ homo.T).T[:, :3]
            # Transform to ego
            ego_points = self.world_to_ego(points_world, ego_pose)

            xyz_all.append(ego_points)
            points_world_all.append(points_world)
            tag_all.append(semantic["obj_tag"])
            id_all.append(semantic["obj_id"])

            # print(f"[debug] lidar {idx} ({key}): ego pts={ego_points.shape[0]}, sem min={semantic['obj_tag'].min()} max={semantic['obj_tag'].max()}")

        # Concatenate all
        if not xyz_all:
            shape = ((np.array(self.pc_range[1::2]) - np.array(self.pc_range[0::2])) / np.array(self.voxel_size)).astype(int)
            # print("[debug] No LiDAR points at all.")
            occ = np.full(shape, 2, dtype=np.uint8)
            sem = np.full(shape, -1, dtype=np.int16)
            obj_id_grid = np.full(shape, -1, dtype=np.int16)
            return occ, sem

        points = np.concatenate(xyz_all, axis=0)
        world_points = np.concatenate(points_world_all, axis=0)
        labels = np.concatenate(tag_all, axis=0)
        obj_ids = np.concatenate(id_all, axis=0)
        # print(f"[debug] world_points = {world_points}")

        # ================= Frame Cache =================
        frame_cache = {
            "ego": {
                "world2ego": ego_pose,
                "ego2world": np.linalg.inv(ego_pose),
            },

            "objects": {},   # obj_id -> object info
            "points": {},    # point-level data
            "sensors": {},   # optional, kept for completeness
        }

        # ---------- Points ----------
        frame_cache["points"] = {
            # Coordinates
            "xyz_world": world_points.copy(),   # (N, 3), world frame
            "xyz_ego": points.copy(),           # (N, 3), ego frame

            # Attributes
            "semantic": labels.copy(),
            "obj_id": obj_ids.copy(),

            # Object-level masks (filled below)
            "object_masks": {},
        }

        # Precompute object masks to avoid repeated comparisons
        unique_obj_ids = np.unique(obj_ids)
        for oid in unique_obj_ids:
            frame_cache["points"]["object_masks"][int(oid)] = (obj_ids == oid)

        # ---------- Objects ----------
        for b in bbox:
            obj_id = int(b.get("id", 0))

            # Parse semantic tag (robust to nested list)
            tag = b["semantic_tags"][0] if len(b["semantic_tags"]) > 0 else 0
            while isinstance(tag, list):
                tag = tag[0]

            is_dynamic = CARLA_SEMANTIC_LABELS.get(tag, ("unknown", None, False))[2]

            # -------- world2obj: fuzzy match + fallback --------
            world2obj = self.build_world2obj_from_bbox(b)
            obj2world = None
            if world2obj is not None:
                obj2world = np.linalg.inv(world2obj)

            frame_cache["objects"][obj_id] = {
                "semantic_tag": tag,
                "is_dynamic": is_dynamic,
                "world2obj": world2obj,
                "obj2world": obj2world,
            }
            
            for key in b:
                frame_cache["objects"][obj_id][key] = b[key]

        # ---------- Sensors (optional, unchanged) ----------
        for key in lidar_keys:
            w2s = np.array(sensors_anno[key]["world2lidar"], dtype=np.float32)
            frame_cache["sensors"][key] = {
                "world2sensor": w2s,
                "sensor2world": np.linalg.inv(w2s),
            }

        # Save cache
        self.frame_cache = frame_cache
        self.frame_cache_queue.append(frame_cache)
        
        # -------------------- static_cache --------------------
        points_static_mask = []

        for idx, pid in enumerate(obj_ids):
            obj_info = frame_cache["objects"].get(int(pid))
            if obj_info is None and pid == 0:
                points_static_mask.append(True)
            elif obj_info is not None and obj_info["is_dynamic"] == False and pid == 0:
                points_static_mask.append(True)
            else:
                points_static_mask.append(False)

        points_static_mask = np.array(points_static_mask, dtype=bool)

        if np.any(points_static_mask):
            static_frame = {
                "ego": {
                    "world2ego": ego_pose.copy(),
                    "ego2world": np.linalg.inv(ego_pose),
                },
                "points": {
                    "xyz_world": world_points[points_static_mask].copy(),
                    "semantic": labels[points_static_mask].copy(),
                    "obj_id": obj_ids[points_static_mask].copy(),
                }
            }
            self.static_cache.append(static_frame)
            # print(f"[debug] static_cache: added {static_frame['points']['xyz_world'].shape[0]} points")

        # ================= debug: semantic summary =================
        # print("[debug] semantic summary:")
        # unique_tags = np.unique(labels)
        # for tag in unique_tags:
        #     mask = labels == tag
        #     pts = int(mask.sum())
        #     ids = np.unique(obj_ids[mask]).tolist()
        #     name, _, is_dynamic = CARLA_SEMANTIC_LABELS.get(tag, ("unknown", None, False))
        #     dyn_str = "dynamic" if is_dynamic else "static"
        #     print(f"  tag={tag:2d} {name:12s} {dyn_str:8s} pts={pts:<6d} id={ids}")

        
        # ================= Range filter =================
        mask = (
            (points[:,0]>=self.pc_range[0]) & (points[:,0]<=self.pc_range[1]) &
            (points[:,1]>=self.pc_range[2]) & (points[:,1]<=self.pc_range[3]) &
            (points[:,2]>=self.pc_range[4]) & (points[:,2]<=self.pc_range[5])
        )
        points = points[mask]
        labels = labels[mask]
        obj_ids = obj_ids[mask]

        # ================= Densify with historical static points =================
        if self.frame_cache_queue:
            static_points_list = []
            static_labels_list = []
            static_obj_ids_list = []

            for prev_cache in self.frame_cache_queue:
                prev_points = prev_cache["points"]["xyz_world"]
                prev_labels = prev_cache["points"]["semantic"]
                prev_obj_ids = prev_cache["points"]["obj_id"]

                static_mask = []
                for pid in prev_obj_ids:
                    obj_info = prev_cache["objects"].get(int(pid))
                    if obj_info is not None and obj_info["is_dynamic"] == True:
                        static_mask.append(False)
                    else:
                        static_mask.append(True)
                static_mask = np.array(static_mask, dtype=bool)

                if np.any(static_mask):
                    static_points_world = prev_points[static_mask]
                    static_labels = prev_labels[static_mask]
                    static_obj_ids = prev_obj_ids[static_mask]
                    # print(f"[debug] static_points_world = {static_points_world}")

                    # transfer coord
                    ego_points_static = self.world_to_ego(static_points_world, ego_pose)
                    # print(f"[debug] ego_points_static = {ego_points_static}")

                    static_points_list.append(ego_points_static)
                    static_labels_list.append(static_labels)
                    static_obj_ids_list.append(static_obj_ids)

            if static_points_list:
                # concat
                points = np.concatenate([points] + static_points_list, axis=0)
                labels = np.concatenate([labels] + static_labels_list, axis=0)
                obj_ids = np.concatenate([obj_ids] + static_obj_ids_list, axis=0)

                # print(f"[debug] fused {sum([p.shape[0] for p in static_points_list])} historical cached static points")
        
        dynamic_points_list = []
        dynamic_labels_list = []
        dynamic_obj_ids_list = []

        current_cache = self.frame_cache
        ego2world_now = current_cache["ego"]["ego2world"]
        world2ego_now = current_cache["ego"]["world2ego"]

        for obj_id, obj_info in current_cache["objects"].items():
            if not obj_info["is_dynamic"]:
                continue

            # Must exist in current LiDAR & pc_range
            mask_now = current_cache["points"]["object_masks"].get(obj_id)
            if mask_now is None or not np.any(mask_now):
                continue

            # Optional: pc_range pruning
            pts_now = current_cache["points"]["xyz_ego"][mask_now]
            in_range = (
                (pts_now[:,0] >= self.pc_range[0]) & (pts_now[:,0] <= self.pc_range[1]) &
                (pts_now[:,1] >= self.pc_range[2]) & (pts_now[:,1] <= self.pc_range[3]) &
                (pts_now[:,2] >= self.pc_range[4]) & (pts_now[:,2] <= self.pc_range[5])
            )
            if not np.any(in_range):
                continue

            # Current pose
            # print(f"[debug] aggregating {obj_id}")
            obj2world_now = obj_info["obj2world"]

            # Traverse history
            for prev_cache in list(self.frame_cache_queue)[:-1]:
                prev_obj = prev_cache["objects"].get(obj_id)
                if prev_obj is None:
                    continue

                mask_prev = prev_cache["points"]["object_masks"].get(obj_id)
                if mask_prev is None or not np.any(mask_prev):
                    continue

                # points in prev world frame
                pts_world_prev = prev_cache["points"]["xyz_world"][mask_prev]
                labels_prev = prev_cache["points"]["semantic"][mask_prev]
                obj_id_prev = prev_cache["points"]["obj_id"][mask_prev]

                # world_prev → obj_local_prev
                world2obj_prev = prev_obj["world2obj"]
                homo = np.hstack([pts_world_prev, np.ones((pts_world_prev.shape[0],1))])
                pts_obj_local = (world2obj_prev @ homo.T).T[:, :3]

                # obj_local → world_now
                homo = np.hstack([pts_obj_local, np.ones((pts_obj_local.shape[0],1))])
                pts_world_now = (obj2world_now @ homo.T).T[:, :3]

                # world_now → ego_now
                homo = np.hstack([pts_world_now, np.ones((pts_world_now.shape[0],1))])
                pts_ego_now = (world2ego_now @ homo.T).T[:, :3]

                dynamic_points_list.append(pts_ego_now)
                dynamic_labels_list.append(labels_prev)
                dynamic_obj_ids_list.append(obj_id_prev)

        if dynamic_points_list:
            points = np.concatenate([points] + dynamic_points_list, axis=0)
            labels = np.concatenate([labels] + dynamic_labels_list, axis=0)
            obj_ids = np.concatenate([obj_ids] + dynamic_obj_ids_list, axis=0)

            # print(f"[debug] fused {sum(p.shape[0] for p in dynamic_points_list)} dynamic historical points")

        shape = ((np.array(self.pc_range[1::2]) - np.array(self.pc_range[0::2])) / np.array(self.voxel_size)).astype(int)
        if points.shape[0]==0:
            # print("[debug] All points out of pc_range.")
            occ = np.full(shape, 2, dtype=np.uint8)
            sem = np.full(shape, -1, dtype=np.int16)
            obj_id_grid = np.full(shape, -1, dtype=np.int16)
            return occ, sem
        
        # -------------------- fuse permanent static_cache --------------------
        static_points_list = []
        static_labels_list = []
        static_obj_ids_list = []

        if self.static_cache:
            # Current ego vehicle world position
            ego_pos_now = np.linalg.inv(ego_pose)[:3, 3]  # ego in world coordinates
            last_fuse_pos = ego_pos_now.copy()
            accumulated_length = 0.0

            index_debug = 0
            # Iterate static_cache in reverse order (from most recent frame to oldest)
            for cache_frame in reversed(self.static_cache):
                ego_pos_frame = cache_frame["ego"]["ego2world"][:3, 3]  # ego position of this frame in world

                # Compute distance from last fused position
                dist = np.linalg.norm(ego_pos_frame - last_fuse_pos)
                # print(f"[debug] dist = {dist}")
                if dist < self.static_step:
                    # Skip this frame if distance is smaller than static_step
                    continue
                index_debug += 1
                # print(f"[debug] history lidar {index_debug} is fused.")

                # Fuse this frame
                pts_world = cache_frame["points"]["xyz_world"]
                labels_cache = cache_frame["points"]["semantic"]
                obj_ids_cache = cache_frame["points"]["obj_id"]

                pts_ego = self.world_to_ego(pts_world, ego_pose)
                static_points_list.append(pts_ego)
                static_labels_list.append(labels_cache)
                static_obj_ids_list.append(obj_ids_cache)

                # Update last fused position
                last_fuse_pos = ego_pos_frame.copy()
                accumulated_length += dist
                # print(f"[debug] assumulated length updated to {accumulated_length}.")

                # Stop if accumulated distance exceeds static_length
                if accumulated_length >= self.static_length:
                    break

        if static_points_list:
            points = np.concatenate([points] + static_points_list, axis=0)
            labels = np.concatenate([labels] + static_labels_list, axis=0)
            obj_ids = np.concatenate([obj_ids] + static_obj_ids_list, axis=0)
            total_pts = sum(p.shape[0] for p in static_points_list)
            # print(f"[debug] fused {total_pts} points from permanent static_cache (sparse fuse)")

        # ================= Voxelization =================
        points_t = torch.from_numpy(points).float()
        labels_t = torch.from_numpy(labels.astype(np.int32)).long()
        obj_ids_t = torch.from_numpy(obj_ids.astype(np.int32)).long()

        min_bound_t = torch.tensor(self.pc_range[0::2], dtype=torch.float32)
        intervals = torch.tensor(self.voxel_size, dtype=torch.float32)
        shape_t = ((torch.tensor(self.pc_range[1::2], dtype=torch.float32) - min_bound_t) / intervals).to(torch.int64)

        occ = torch.full(shape_t.tolist(), 0, dtype=torch.uint8)
        sem = torch.full(shape_t.tolist(), -1, dtype=torch.int16)
        obj_id_grid = torch.full(shape_t.tolist(), -1, dtype=torch.int32)  # new: voxel-level obj_id

        # compute voxel coordinates
        coor = ((points_t - min_bound_t - 1e-5)/intervals).to(torch.int64)
        # print(f"[debug] before filtering out, coor min: {coor.min(dim=0).values.tolist()}, max: {coor.max(dim=0).values.tolist()}")

        # filter out-of-bound voxels
        mask = (
            (coor[:, 0] >= 0) & (coor[:, 0] < shape_t[0]) &
            (coor[:, 1] >= 0) & (coor[:, 1] < shape_t[1]) &
            (coor[:, 2] >= 0) & (coor[:, 2] < shape_t[2])
        )
        coor = coor[mask]
        labels_t = labels_t[mask]
        obj_ids_t = obj_ids_t[mask]

        # remove duplicates: keep one voxel per position
        ranks = coor[:,2]*shape_t[0]*shape_t[1] + coor[:,1]*shape_t[0] + coor[:,0]
        order = ranks.argsort()
        coor = coor[order]
        labels_t = labels_t[order]
        obj_ids_t = obj_ids_t[order]
        ranks = ranks[order]
        keep = torch.ones(coor.shape[0], dtype=torch.bool)
        keep[1:] = ranks[1:] != ranks[:-1]
        coor = coor[keep]
        labels_t = labels_t[keep]
        obj_ids_t = obj_ids_t[keep]

        # fill voxel grids
        occ[coor[:,1], coor[:,0], coor[:,2]] = 1
        sem[coor[:,1], coor[:,0], coor[:,2]] = labels_t.to(torch.int16)
        obj_id_grid[coor[:,1], coor[:,0], coor[:,2]] = obj_ids_t.to(torch.int32)
        
        # print(f"[debug] voxelized sem min={sem.min().item()} max={sem.max().item()}")

        # ================= Dynamic Object Ray-based Filling =================
        for obj_id, obj_info in current_cache["objects"].items():
            if not obj_info["is_dynamic"]:
                continue

            # Skip if no center/extent info (cannot reliably do bbox filling)
            if "center" not in obj_info or "extent" not in obj_info or "rotation" not in obj_info:
                continue

            mask_obj_voxels = (obj_id_grid == obj_id)
            if not mask_obj_voxels.any():
                continue
            
            points_obj_mask = (current_cache["points"]["obj_id"] == obj_id)
            if not points_obj_mask.any():
                continue
            labels_obj = current_cache["points"]["semantic"][points_obj_mask].tolist()
            obj_label = Counter(labels_obj).most_common(1)[0][0]
            
            if obj_label not in FILLED_DYNAMIC_LABELS:
                continue
            
            # print(f"[debug] prepare to fill object, obj_label = {obj_label}, obj_id = {obj_id}")
            
            occ, sem = self.fill_dynamic_object_voxel(
                occ=occ,
                sem=sem,
                voxel_size=torch.tensor(self.voxel_size, device=occ.device),
                pc_range=torch.tensor(self.pc_range, device=occ.device),
                world2ego=current_cache["ego"]["world2ego"],
                obj_center=obj_info["center"],
                obj_extent=obj_info["extent"],
                obj_yaw=obj_info["rotation"][2],
                obj_label=obj_label
            )
                        
        # print(f"[debug] after filling, voxelized sem min={sem.min().item()} max={sem.max().item()}")

        # ---------------- ground semantic fix ----------------
        ground_labels = [1, 2, 10, 24, 25]  # road, sidewalk, terrain, road_line, ground
        ground_labels_t = torch.tensor(ground_labels, device=sem.device)

        Y, X, Z = sem.shape
        tile_size = int(np.sqrt(self.ground_flood_threshold))  # tile size
        t0 = time.time() # debug timing start point

        for z in range(Z):
            slice_sem = sem[:, :, z]
            slice_occ = occ[:, :, z]

            # ground voxels in the mask
            mask_ground = (slice_occ == 1) & torch.isin(slice_sem, ground_labels_t)
            if not mask_ground.any():
                continue

            mask_free = (slice_occ == 0)
            visited = np.zeros_like(mask_free, dtype=bool)

            # divide z-level into tiles
            for ty in range(0, Y, tile_size):
                for tx in range(0, X, tile_size):
                    y1 = min(ty + tile_size, Y)
                    x1 = min(tx + tile_size, X)

                    # any ground voxels in the mask?
                    tile_has_ground = mask_ground[ty:y1, tx:x1].any()
                    if not tile_has_ground:
                        visited[ty:y1, tx:x1] = True
                        continue

                    # BFS within a tile
                    for y in range(ty, y1):
                        for x in range(tx, x1):
                            if not mask_free[y, x] or visited[y, x]:
                                continue

                            queue = deque([(y, x)])
                            curr_visited = np.zeros_like(mask_free, dtype=bool)
                            curr_visited[y, x] = True

                            region = [(y, x)]
                            boundary_labels = []
                            is_touch_boundary = False
                            is_touch_previous_vis = False
                            too_large = False

                            while queue:
                                cy, cx = queue.popleft()

                                if len(region) > self.ground_flood_threshold:
                                    too_large = True
                                    break

                                for ny, nx in [(cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1)]:

                                    # out of boundary
                                    if not (0 <= ny < Y and 0 <= nx < X):
                                        is_touch_boundary = True
                                        continue

                                    # previously visited → hollow area
                                    if visited[ny, nx]:
                                        is_touch_previous_vis = True
                                        break

                                    # current vis
                                    if curr_visited[ny, nx]:
                                        continue

                                    # occ == 1 → collect boundary
                                    if slice_occ[ny, nx] == 1:
                                        curr_visited[ny, nx] = True
                                        if slice_sem[ny, nx].item() in ground_labels:
                                            boundary_labels.append(slice_sem[ny, nx].item())
                                        continue

                                    # free voxel. continue expanding
                                    curr_visited[ny, nx] = True
                                    queue.append((ny, nx))
                                    region.append((ny, nx))

                                if is_touch_previous_vis:
                                    break

                            # ==================================================
                            # After BFS finished:update all visited
                            # ==================================================
                            visited[curr_visited] = True

                            # ==================================================
                            # Whether to fill
                            # ==================================================
                            if (not too_large) and boundary_labels and (not is_touch_previous_vis):
                                fill_label = Counter(boundary_labels).most_common(1)[0][0]
                                for ry, rx in region:
                                    sem[ry, rx, z] = fill_label
                                    occ[ry, rx, z] = 1
                                    obj_id_grid[ry, rx, z] = 0

        # print(f"[debug][ground] after flood-fill z-loop: {time.time() - t0:.3f}s")
        
        # ---------------- remove noises above ground, keep lower ones only ----------------
        
        is_ground = torch.isin(sem, ground_labels_t) & (occ == 1)

        ground_z_mask = torch.zeros_like(is_ground)
        seen_ground = torch.zeros((Y, X), dtype=torch.bool, device=sem.device)

        for z in range(Z):
            first_ground = is_ground[:, :, z] & (~seen_ground)
            ground_z_mask[:, :, z] = first_ground
            seen_ground |= is_ground[:, :, z]

        noise_ground = is_ground & (~ground_z_mask)
        num_noise = noise_ground.sum().item()

        if num_noise > 0:
            sem[noise_ground] = -1
            occ[noise_ground] = 0
            obj_id_grid[noise_ground] = -1

        # print(f"[debug][ground] removed floating ground voxels: {num_noise}")
        # print(f"[debug][ground] column denoise done: {time.time() - t0:.3f}s")

       # ---------------- fill ground downwards ----------------
        t1 = time.time()

        is_ground = torch.isin(sem, ground_labels_t) & (occ == 1)
        is_blocker = (occ == 1) & (~torch.isin(sem, ground_labels_t))

        can_fill = torch.zeros((Y, X), dtype=torch.bool, device=sem.device)
        last_ground_sem = torch.full((Y, X), -1, dtype=sem.dtype, device=sem.device)

        # Z-1 -> 0,
        for z in range(Z - 1, -1, -1):
            # update source
            ground_here = is_ground[:, :, z]
            last_ground_sem[ground_here] = sem[:, :, z][ground_here]
            can_fill |= ground_here

            # blocker ends filling
            can_fill &= ~is_blocker[:, :, z]

            # positions need to be filled in this layer
            fill_mask = can_fill & (occ[:, :, z] == 0)
            if fill_mask.any():
                sem[:, :, z][fill_mask] = last_ground_sem[fill_mask]
                occ[:, :, z][fill_mask] = 1
                obj_id_grid[:, :, z][fill_mask] = 0

        # print(f"[debug][ground] after downward fill (fast): {time.time() - t1:.3f}s")
        
        
        # ================= hollow fix, coarse-to-fine =================
        occ, sem, obj_id_grid = self.hollow_fix(occ, sem, obj_id_grid, stride=4)
        
        self.downward_fill(occ, sem, obj_id_grid)

        # print(f"[debug] after ground adjustment, voxelized sem min={sem.min().item()} max={sem.max().item()}")

        return occ.numpy(), sem.numpy()