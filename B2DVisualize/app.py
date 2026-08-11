import os
import shutil
import yaml
from flask import Flask, render_template, request, send_from_directory, abort, url_for, jsonify
import json, gzip, html
import importlib
from core.loader import DatasetLoader

app = Flask(__name__)

# Load dataset configuration from YAML file
CONFIG = yaml.safe_load(open("config/dataset_config_local.yaml"))

VISUALIZER_CONFIG = {
    "json": "visualizers.json_vis.JsonVisualizer",
    "image": "visualizers.image_vis.ImageVisualizer",
    "depth": "visualizers.depth_vis.DepthVisualizer",
    "lidar": "visualizers.lidar_vis.LidarVisualizer",
    "radar": "visualizers.radar_vis.RadarVisualizer",
    "bev": "visualizers.bev_vis.BEVVisualizer",
    "semantic": "visualizers.semantic_vis.SemanticVisualizer",
    "lidar_semantic": "visualizers.lidar_semantic_vis.SemanticLidarVisualizer",
    "occ": "visualizers.occ_vis.Occupancy3DVisualizer"
}

def get_visualizer(v_type):
    """Dynamically import visualizer class and instantiate."""
    path = VISUALIZER_CONFIG[v_type]
    module_path, class_name = path.rsplit(".", 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    return cls()

# Initialize dataset loader with configuration
loader = DatasetLoader(CONFIG)

# Cache directory for temporarily storing files (e.g., images)
CACHE_DIR = ".cache"
os.makedirs(CACHE_DIR, exist_ok=True)

# Add a "name" field to each module configuration (used for caching and identification)
for k, v in CONFIG["directories"].items():
    v["name"] = k


def cache_file(module_name, abs_path):
    """
    Cache a file (e.g., image or other asset) into CACHE_DIR and
    return a relative URL that can be directly accessed by the frontend.

    Args:
        module_name (str): Name of the module (used as cache key).
        abs_path (str): Absolute path of the source file.

    Returns:
        str: Relative URL path to the cached file.
    """
    # Preserve the original file extension
    ext = os.path.splitext(abs_path)[1]
    cached_name = f"{module_name}{ext}"
    cached_path = os.path.join(CACHE_DIR, cached_name)

    # Copy the file into the cache directory
    shutil.copyfile(abs_path, cached_path)

    # Return a URL path that maps to the cached file
    return f"/cache/{cached_name}"


@app.route("/")
def index():
    """
    Main index page.
    - Lists available scenes and indices
    - Handles scene/index selection and navigation (prev/next)
    - Renders each module using its corresponding visualizer
    """

    # ---------- Helper: find first valid scene ----------
    def find_first_valid_scene(scenes, loader):
        for s in scenes:
            indices = loader.list_indices(s)
            if indices:
                return s, indices
        return None, None

    # ---------- List scenes ----------
    scenes = loader.list_scenes()
    if not scenes:
        return "No scenes found", 404

    # ---------- Scene selection ----------
    requested_scene = request.args.get("scene")

    if requested_scene in scenes:
        indices = loader.list_indices(requested_scene)
        if indices:
            scene = requested_scene
        else:
            scene, indices = find_first_valid_scene(scenes, loader)
    else:
        scene, indices = find_first_valid_scene(scenes, loader)

    if scene is None or not indices:
        return "No data available in any scene", 404

    # ---------- Index selection ----------
    try:
        idx = int(request.args.get("index", indices[0]))
    except (ValueError, TypeError):
        idx = indices[0]

    if idx not in indices:
        idx = indices[0]

    # ---------- Prev / Next navigation ----------
    action = request.args.get("action")
    if action in ("prev", "next"):
        idx_pos = indices.index(idx)
        if action == "prev":
            idx = indices[idx_pos - 1] if idx_pos > 0 else indices[-1]
        elif action == "next":
            idx = indices[idx_pos + 1] if idx_pos < len(indices) - 1 else indices[0]

    # ---------- Load data for scene + index ----------
    files = loader.get_item(scene, idx)

    # ---------- Render each module ----------
    rendered = {}
    for name, entry in files.items():
        vis = get_visualizer(entry["visualizer"])

        if entry["visualizer"] == "json":
            initial_filter = request.args.get(f"filter_{name}", "")
            rendered[name] = vis.render(entry, cache_file, initial_filter=initial_filter)
        else:
            rendered[name] = vis.render(entry, cache_file)

    print("[debug] rendered keys:", rendered.keys())

    # ---------- Render page ----------
    return render_template(
        "index.html",
        scenes=scenes,
        scene=scene,
        indices=indices,
        current_index=idx,
        rendered=rendered,
        layout=CONFIG["layout"]
    )



@app.route("/filter", methods=["POST"])
def filter_module():
    """
    API endpoint for filtering JSON modules by keywords.
    This is typically triggered by frontend interactions.
    """
    data = request.json

    scene = data.get("scene")
    index = int(data.get("index"))
    module_name = data.get("module_name")
    keywords = data.get("keywords", [])

    if not scene or module_name is None:
        return jsonify({"error": "Missing parameters"}), 400

    # Re-fetch the module entry via the loader (authoritative source)
    files = loader.get_item(scene, index)

    if module_name not in files:
        return jsonify({"error": f"Module {module_name} not found"}), 400

    entry = files[module_name]
    abs_path = entry["abs_path"]

    # Load JSON data (supports both plain .json and .json.gz)
    if abs_path.endswith(".gz"):
        with gzip.open(abs_path, "rt", encoding="utf-8") as f:
            json_data = json.load(f)
    else:
        with open(abs_path, "r", encoding="utf-8") as f:
            json_data = json.load(f)

    # Apply JSON tree preprocessing and keyword-based filtering
    from visualizers.json_vis import append_list_id, filter_json_tree
    json_data = append_list_id(json_data)
    filtered_data = filter_json_tree(json_data, keywords)

    # Return filtered JSON as escaped HTML
    html_content = f"<pre>{html.escape(json.dumps(filtered_data, indent=2))}</pre>"
    return jsonify({"html": html_content})


@app.route("/cache/<path:filename>")
def cache(filename):
    """
    Serve cached files from CACHE_DIR.
    """
    cached_path = os.path.join(CACHE_DIR, filename)
    if not os.path.exists(cached_path):
        abort(404)
    return send_from_directory(CACHE_DIR, filename)


if __name__ == "__main__":
    # Run Flask app in debug mode, accessible from all network interfaces
    app.run(host="0.0.0.0", port=8080, debug=True)
