import os
import numpy as np
import matplotlib.pyplot as plt
from util.webutil import get_proxy_url

from visualizers.semantic_vis import (
    CARLA_SEMANTIC_LABELS,
    build_semantic_legend_html
)

class SemanticLidarVisualizer:
    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        module_name = file_entry["name"]

        # ---------- Load NPZ ----------
        data = np.load(abs_path)

        if "xyz" not in data or "obj_tag" not in data:
            return "<p style='color:red'>Invalid semantic lidar npz format</p>"

        xyz = data["xyz"]        # (N, 3)
        labels = data["obj_tag"].astype(np.int32)  # semantic labels

        x = xyz[:, 0]
        y = xyz[:, 1]

        # ---------- BEV range ----------
        bev_range = 64.0
        mask = (
            (x > -bev_range) & (x < bev_range) &
            (y > -bev_range) & (y < bev_range)
        )

        fb = x[mask]
        lr = y[mask]
        labels = labels[mask]

        if labels.size == 0:
            return "<p>No LiDAR points in BEV range</p>"

        # ---------- Label → color ----------
        colors = np.zeros((labels.shape[0], 4), dtype=np.float32)

        for label_id, (_, rgb) in CARLA_SEMANTIC_LABELS.items():
            idx = labels == label_id
            if np.any(idx):
                colors[idx, :3] = np.array(rgb, dtype=np.float32) / 255.0
                colors[idx, 3] = 1.0

        # unknown labels → white
        unknown = colors[:, 3] == 0
        colors[unknown] = [1.0, 1.0, 1.0, 1.0]

        # ---------- Plot ----------
        fig, ax = plt.subplots(figsize=(6, 6), dpi=150)
        ax.scatter(
            lr,
            fb,
            c=colors,
            s=1,
            linewidths=0
        )

        ax.set_xlim(-bev_range, bev_range)
        ax.set_ylim(-bev_range, bev_range)
        ax.set_aspect("equal")
        ax.set_xlabel("Left / Right (m)")
        ax.set_ylabel("Backward / Forward (m)")
        ax.set_title("Semantic LiDAR BEV (±64m)")
        ax.grid(True, linestyle="--", linewidth=0.3, alpha=0.5)

        # ---------- Save ----------
        os.makedirs(".cache", exist_ok=True)
        cache_png = os.path.join(".cache", f"{module_name}_semantic_lidar.png")
        plt.tight_layout()
        plt.savefig(cache_png)
        plt.close(fig)

        url = get_proxy_url("cache", filename=os.path.basename(cache_png))

        # ---------- Legend ----------
        present_labels = set(np.unique(labels).tolist())
        legend_html = build_semantic_legend_html(
            CARLA_SEMANTIC_LABELS,
            present_labels=present_labels
        )

        return f"""
        <div style="display:flex; gap:16px;">
            <div style="flex:3;">
                <p><b>Semantic LiDAR BEV</b> (points: {labels.shape[0]})</p>
                <img src="{url}" style="max-width:100%;" />
            </div>
            <div style="flex:1;">
                {legend_html}
            </div>
        </div>
        """
