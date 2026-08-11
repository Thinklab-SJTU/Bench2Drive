import os
import numpy as np
from PIL import Image
import matplotlib.cm as cm
from util.webutil import get_proxy_url


class BEVVisualizer:
    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]

        # Load npz
        with np.load(abs_path) as data:
            key = data.files[0]
            bev = data[key]

        if bev.ndim != 3:
            return f"<p style='color:red'>Invalid BEV shape: {bev.shape}</p>"

        # ----------------------------------
        # Normalize shape to (C, H, W)
        # Most birdview masks are (H, W, C)
        # ----------------------------------
        if bev.shape[-1] <= 64:
            bev = np.transpose(bev, (2, 0, 1))

        C, H, W = bev.shape

        cmap = cm.get_cmap("tab20", C)

        html = []
        html.append(
            f"<p><b>BEV:</b> {C} channels, size {H}×{W}</p>"
        )

        for i in range(C):
            mask = bev[i]

            # Normalize for visualization
            if mask.max() > mask.min():
                norm = (mask - mask.min()) / (mask.max() - mask.min())
            else:
                norm = np.zeros_like(mask)

            # FIX: tuple -> numpy array
            color = (np.array(cmap(i)[:3]) * 255).astype(np.uint8)
            img = (norm[..., None] * color).astype(np.uint8)

            fname = f"{file_entry['name']}_ch{i}.png"
            fpath = os.path.join(".cache", fname)
            Image.fromarray(img).save(fpath)

            url = get_proxy_url("cache", filename=fname)

            html.append(f"""
                <div style="display:inline-block; margin:4px; text-align:center;">
                    <div style="font-size:12px;">channel {i}</div>
                    <img src="{url}" width="128" height="128"/>
                </div>
            """)

        return "<div>" + "\n".join(html) + "</div>"
