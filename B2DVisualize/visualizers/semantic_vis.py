import os
import numpy as np
from PIL import Image
import cv2
from util.webutil import get_proxy_url

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


def build_semantic_legend_html(label_dict, present_labels=None):
    """
    label_dict: {id: (name, (r,g,b))}
    present_labels: set or list of label ids (可选，只显示当前图中出现的类)
    """
    items = []

    for label_id in sorted(label_dict.keys()):
        if present_labels is not None and label_id not in present_labels:
            continue

        name, (r, g, b) = label_dict[label_id]
        color = f"rgb({r},{g},{b})"

        items.append(f"""
        <div style="
            display:flex;
            align-items:center;
            margin-bottom:4px;
            font-size:12px;
        ">
            <div style="
                width:16px;
                height:16px;
                background:{color};
                margin-right:8px;
                border:1px solid #333;
            "></div>
            <div>
                <b>{label_id}</b> : {name}
            </div>
        </div>
        """)

    return f"""
    <div style="
        max-height:300px;
        overflow-y:auto;
        padding:8px;
        border:1px solid #ccc;
        background:#fafafa;
    ">
        <b>Semantic Legend</b>
        <div style="margin-top:6px;">
            {''.join(items)}
        </div>
    </div>
    """

class SemanticVisualizer:
    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        module_name = file_entry["name"]

        raw = cv2.imread(abs_path, cv2.IMREAD_UNCHANGED)
        if raw is None:
            return "<p style='color:red'>Failed to load semantic image</p>"

        if raw.ndim == 2:
            sem_id = raw
        else:
            sem_id = raw[:, :, 2]  # R channel

        h, w = sem_id.shape
        color_img = np.zeros((h, w, 3), dtype=np.uint8)

        for label_id, (_, color) in CARLA_SEMANTIC_LABELS.items():
            mask = sem_id == label_id
            if np.any(mask):
                color_img[mask] = color

        # save
        cache_png = os.path.join(".cache", f"{module_name}_semantic.png")
        Image.fromarray(color_img).save(cache_png)
        url = get_proxy_url("cache", filename=os.path.basename(cache_png))

        # ---------- legend ----------
        present_labels = set(np.unique(sem_id).tolist())
        legend_html = build_semantic_legend_html(
            CARLA_SEMANTIC_LABELS,
            present_labels=present_labels
        )

        return f"""
        <div style="display:flex; gap:16px;">
            <div style="flex:3;">
                <img src="{url}" style="max-width:100%;" />
            </div>
            <div style="flex:1;">
                {legend_html}
            </div>
        </div>
        """
