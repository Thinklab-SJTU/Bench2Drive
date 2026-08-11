import os
import numpy as np
import laspy
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from util.webutil import get_proxy_url

class LidarVisualizer:
    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        module_name = file_entry["name"]

        # ---------- Load LAZ/LAS ----------
        with laspy.open(abs_path) as f:
            points = f.read()

        x = points.x
        y = points.y
        z = points.z

        # ---------- Get point attributes ----------
        dims = list(points.point_format.dimension_names)  # generator -> list
        print("LiDAR point attributes:", dims)

        has_intensity = "intensity" in dims
        if has_intensity:
            intensity = points.intensity.astype(np.float32)
            print("Intensity stats: min =", intensity.min(), "max =", intensity.max())
        else:
            intensity = None
            print("No intensity field found, using default alpha=1")

        # ---------- Print first 3 points ----------
        print("-" * 60)
        for i in range(min(3, len(points))):
            print(f"Point {i}:")
            for d in dims:
                val = getattr(points, d)[i]
                # SubFieldView -> 普通类型
                if hasattr(val, 'tolist'):
                    val = val.tolist()
                print(f"  {d}: {val}")
            print("-" * 60)

        # ---------- BEV range ----------
        bev_range = 64.0
        mask = (
            (x > -bev_range) & (x < bev_range) &
            (y > -bev_range) & (y < bev_range)
        )

        fb = x[mask]
        lr = y[mask]
        z_masked = z[mask]

        if has_intensity:
            intensity_masked = intensity[mask]
            use_intensity = np.any(intensity_masked > 0)
        else:
            intensity_masked = None
            use_intensity = False

        # ---------- Prepare RGBA colors ----------
        norm = Normalize(vmin=z_masked.min(), vmax=z_masked.max())
        cmap = cm.get_cmap("viridis")
        colors = cmap(norm(z_masked))  # RGBA (N,4)

        if use_intensity:
            alpha = np.clip(intensity_masked / 65535.0, 0.05, 1.0)
        else:
            alpha = np.ones_like(z_masked)

        colors[:, 3] = alpha  # overwrite alpha channel

        # ---------- Plot ----------
        fig, ax = plt.subplots(figsize=(6, 6), dpi=150)
        sc = ax.scatter(
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
        ax.set_title("LiDAR BEV (±64m)")
        ax.grid(True, linestyle="--", linewidth=0.3, alpha=0.5)
        plt.colorbar(sc, ax=ax, fraction=0.046, pad=0.04, label="Height (m)")

        # ---------- Save to cache ----------
        os.makedirs(".cache", exist_ok=True)
        cache_png = os.path.join(".cache", f"{module_name}.png")
        plt.tight_layout()
        plt.savefig(cache_png)
        plt.close(fig)

        url = get_proxy_url("cache", filename=f"{module_name}.png")

        return f"""
        <div>
            <p><b>LiDAR BEV</b> (points: {x.shape[0]})</p>
            <img src="{url}" style="max-width:100%;" />
        </div>
        """
