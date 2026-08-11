import os
import numpy as np
import matplotlib.pyplot as plt
from util.webutil import get_proxy_url

from visualizers.semantic_vis import CARLA_SEMANTIC_LABELS

CARLA_SEMANTIC_LABELS = {
    0: ("unlabeled", (0, 0, 0)),
    1: ("road", (128, 64, 128)),
    2: ("sidewalk", (244, 35, 232)),
    3: ("building", (70, 70, 70)),
    4: ("wall", (102, 102, 156)),
    5: ("fence", (190, 153, 153)),
    6: ("pole", (153, 153, 153)),
    7: ("traffic_light", (250, 170, 30)),
    8: ("traffic_sign", (220, 220, 0)),
    9: ("vegetation", (107, 142, 35)),
    10: ("terrain", (152, 251, 152)),
    11: ("sky", (70, 130, 180)),
    12: ("pedestrian", (220, 20, 60)),
    13: ("rider", (255, 0, 0)),
    14: ("car", (0, 0, 142)),
    15: ("truck", (0, 0, 70)),
    16: ("bus", (0, 60, 100)),
    17: ("train", (0, 80, 100)),
    18: ("motorcycle", (0, 0, 230)),
    19: ("bicycle", (119, 11, 32)),
    20: ("static", (110, 190, 160)),
    21: ("dynamic", (170, 120, 50)),
    22: ("other", (55, 90, 80)),
    23: ("water", (45, 60, 150)),
    24: ("road_line", (157, 234, 50)),
    25: ("ground", (81, 0, 81)),
    26: ("bridge", (150, 100, 100)),
    27: ("rail_track", (230, 150, 140)),
    28: ("guard_rail", (180, 165, 180)),
}

class Occupancy3DVisualizer:
    """
    3D visualization of occupancy voxel grid.
    - If only occ exists: show occupied voxels in red
    - If sem exists: color occupied voxels by semantic label
    """

    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        module_name = file_entry["name"]

        # ---------- Load NPZ ----------
        data = np.load(abs_path)

        # ---------- Read occ ----------
        if "occ" in data:
            occ = data["occ"]
        else:
            # backward compatibility
            occ = data[list(data.files)[0]]

        if occ.ndim != 3:
            return "<p style='color:red'>Invalid occ format (expect 3D voxel grid)</p>"

        # ---------- Optional semantic ----------
        sem = data["seg"] if "seg" in data else None

        # ---------- occupied voxels only ----------
        occ_mask = occ == 1
        idx = np.argwhere(occ_mask)  # (N, 3) in (y, x, z)

        if idx.shape[0] == 0:
            return "<p>No occupied voxels</p>"

        # ---------- Convert to metric coordinates ----------
        voxel_size = np.array([0.4, 0.4, 0.4])
        pc_range = np.array([-25.6, -25.6, -5.0])

        # NOTE: idx is (y, x, z)
        points = idx[:, [1, 0, 2]] * voxel_size + pc_range + voxel_size / 2.0
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # ---------- Colors ----------
        if sem is not None:
            sem_vals = sem[occ_mask]

            colors = np.zeros((sem_vals.shape[0], 4), dtype=np.float32)

            for label_id, (_, rgb) in CARLA_SEMANTIC_LABELS.items():
                m = sem_vals == label_id
                if np.any(m):
                    colors[m, :3] = np.array(rgb, dtype=np.float32) / 255.0
                    colors[m, 3] = 1.0

            # unknown / unlabeled semantic → white
            unknown = colors[:, 3] == 0
            colors[unknown] = [1.0, 1.0, 1.0, 1.0]

            title = "3D Occupancy + Semantic"

        else:
            colors = "red"
            title = "3D Occupancy"

        # ---------- Plot ----------
        fig = plt.figure(figsize=(7, 6), dpi=150)
        ax = fig.add_subplot(111, projection="3d")

        ax.scatter(
            y,
            x,
            z,
            s=2,
            c=colors,
            alpha=0.6 if sem is None else 1.0,
            linewidths=0,
        )

        ax.set_xlabel("Y (m)")
        ax.set_ylabel("X (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(title)

        # ---------- Equal aspect ----------
        max_range = np.array([
            x.max() - x.min(),
            y.max() - y.min(),
            z.max() - z.min()
        ]).max() / 2.0

        mid_x = (x.max() + x.min()) * 0.5
        mid_y = (y.max() + y.min()) * 0.5
        mid_z = (z.max() + z.min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # ---------- Save ----------
        os.makedirs(".cache", exist_ok=True)
        cache_png = os.path.join(".cache", f"{module_name}_occ_3d.png")
        plt.tight_layout()
        plt.savefig(cache_png)
        plt.close(fig)

        url = get_proxy_url("cache", filename=os.path.basename(cache_png))

        return f"""
        <div>
            <p><b>{title}</b></p>
            <p>Occupied voxels: {idx.shape[0]}</p>
            <img src="{url}" style="max-width:100%;" />
        </div>
        """
