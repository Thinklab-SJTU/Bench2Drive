import os
import h5py
import numpy as np
import matplotlib.pyplot as plt

from util.webutil import get_proxy_url


class RadarVisualizer:
    """
    Visualize CARLA radar detections in ego-frame BEV.

    Radar point format:
        [depth, azimuth, altitude, velocity]
    """

    RADAR_CONFIGS = {
        "radar_front":       {"x": 2.27, "y": 0.0,  "yaw": 0.0},
        "radar_front_left":  {"x": 1.21, "y": -0.85,"yaw": -90.0},
        "radar_front_right": {"x": 1.21, "y": 0.85, "yaw": 90.0},
        "radar_back_left":   {"x": -2.0, "y": -0.67,"yaw": 180.0},
        "radar_back_right":  {"x": -2.0, "y": 0.67,"yaw": 180.0},
    }

    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        module_name = file_entry["name"]

        with h5py.File(abs_path, "r") as f:
            radar_keys = list(f.keys())

        if not radar_keys:
            return "<p style='color:red'>No radar data</p>"

        bev_range = 64.0
        fig, ax = plt.subplots(figsize=(6, 6))
        fig.patch.set_facecolor("black")  # figure 背景
        ax.set_facecolor("black")         # axes 背景

        all_xy = []
        all_v = []

        # -----------------------------
        # Collect all radar points
        # -----------------------------
        for radar_name in radar_keys:
            if radar_name not in self.RADAR_CONFIGS:
                continue

            cfg = self.RADAR_CONFIGS[radar_name]

            with h5py.File(abs_path, "r") as f:
                pts = np.asarray(f[radar_name])

            if pts.size == 0 or pts.shape[1] < 4:
                continue

            depth    = pts[:, 0]
            azimuth  = pts[:, 1]
            velocity = pts[:, 3]

            # Polar -> local Cartesian
            x_local = depth * np.cos(azimuth)
            y_local = depth * np.sin(azimuth)

            # Local -> ego frame
            theta = np.deg2rad(cfg["yaw"])
            R = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta),  np.cos(theta)],
            ])
            xy_ego = (R @ np.stack([x_local, y_local], axis=0)).T
            xy_ego[:, 0] += cfg["x"]
            xy_ego[:, 1] += cfg["y"]

            all_xy.append(xy_ego)
            all_v.append(velocity)

        if not all_xy:
            return "<p style='color:red'>No valid radar points</p>"

        all_xy = np.vstack(all_xy)
        all_v = np.concatenate(all_v)

        # -----------------------------
        # Velocity color mapping
        # -----------------------------
        vmax = np.percentile(np.abs(all_v), 95) + 3.0
        vmax = max(vmax, 1.0)
        cmap = plt.cm.seismic
        colors = cmap((np.clip(all_v, -vmax, vmax) + vmax) / (2 * vmax))

        ax.scatter(
            all_xy[:, 1],
            all_xy[:, 0],
            s=6,
            c=colors,
            edgecolors="none"
        )

        # Ego origin
        ax.scatter(0, 0, c="white", s=40, marker="x")

        ax.set_xlim(-bev_range, bev_range)
        ax.set_ylim(-bev_range, bev_range)
        ax.set_aspect("equal")

        ax.set_xlabel("Y (right) [m]", color="white")
        ax.set_ylabel("X (forward) [m]", color="white")
        ax.tick_params(colors="white")
        ax.grid(True, linestyle="--", alpha=0.3, color="gray")

        # Colorbar
        sm = plt.cm.ScalarMappable(
            cmap=cmap,
            norm=plt.Normalize(vmin=-vmax, vmax=vmax),
        )
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Radial velocity (m/s)", color="white")
        cbar.ax.yaxis.set_tick_params(color="white")
        plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="white")

        # Save and return URL
        cache_png = os.path.join(".cache", f"{module_name}.png")
        plt.savefig(cache_png, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)

        url = get_proxy_url("cache", filename=f"{module_name}.png")

        return f"""
        <div>
            <p><b>Radar BEV</b> (points: {all_v.shape[0]})</p>
            <img src="{url}" style="max-width:100%;" />
        </div>
        """
