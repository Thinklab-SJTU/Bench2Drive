import numpy as np
from PIL import Image
import os
import matplotlib.cm as cm
from util.webutil import get_proxy_url


class DepthVisualizer:
    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        module_name = file_entry["name"]

        # ---------- Load npz ----------
        with np.load(abs_path) as data:
            keys = list(data.keys())
            depth_data = data[keys[0]]  # assume the first key stores depth in meters

        # ---------- Cyclic depth visualization ----------
        cycle = 50.0  # meters per color cycle
        depth_mod = np.mod(depth_data, cycle) / cycle  # normalized to [0, 1]

        # ---------- Apply colormap ----------
        cmap = cm.plasma  # plasma / magma / viridis
        depth_color = (cmap(depth_mod)[:, :, :3] * 255).astype(np.uint8)

        # ---------- Save directly to cache ----------
        cache_png = os.path.join(".cache", f"{module_name}.png")
        Image.fromarray(depth_color).save(cache_png)

        url = get_proxy_url("cache", filename=f"{module_name}.png")

        # ---------- Render ----------
        key_info = ", ".join(keys)
        return f"""
        <div>
            <p>Depth keys: {key_info}</p>
            <img src="{url}" style="max-width:100%;" />
        </div>
        """
