# B2DVisualize

A lightweight web-based visualizer for [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) data.
It scans a dataset directory, lists available scenes and frames, and renders each sensor / annotation
module (camera, LiDAR, radar, BEV, occupancy, JSON annotation, ...) in a two-column web page.

## Structure

```
B2DVisualize/
├── app.py                      # Flask entry point (run this)
├── config/
│   └── dataset_config.yaml     # Template config — copy it to dataset_config_local.yaml
├── core/
│   └── loader.py               # Scans dataset dir and resolves per-module file paths
├── visualizers/                # One class per data modality
│   ├── json_vis.py             # Raw JSON tree + keyword filter
│   ├── image_vis.py            # Plain images (.jpg/.png)
│   ├── depth_vis.py            # Depth npz → cyclic colormap
│   ├── lidar_vis.py            # LAZ/LAS → BEV scatter (height / intensity)
│   ├── lidar_semantic_vis.py   # Semantic LiDAR npz → colored BEV
│   ├── radar_vis.py            # CARLA radar h5 → ego-frame BEV with velocity
│   ├── bev_vis.py              # Multi-channel BEV masks npz
│   ├── semantic_vis.py         # Semantic segmentation image → CARLA palette
│   └── occ_vis.py              # 3D occupancy npz (+ optional semantics)
├── util/
│   └── webutil.py              # URL helper (proxy-aware)
└── templates/
    └── index.html              # Main page template
```

## Dependencies

```bash
pip install flask pyyaml numpy pillow matplotlib laspy h5py opencv-python
```

(`laspy` is only needed for the `lidar` module, `h5py` for `radar`, `opencv-python` for `semantic`.)

## Configuration

**You must create `config/dataset_config_local.yaml` yourself.** It is the file `app.py` actually
reads at startup, and it is git-ignored so each machine keeps its own copy.

Start by copying the template and editing it to match your dataset:

```bash
cp config/dataset_config.yaml config/dataset_config_local.yaml
```

Then set `dataset.root_dir` to the directory that contains the scene folders, e.g.:

```yaml
dataset:
  root_dir: /path/to/Bench2Drive
```

Scenes are auto-detected as subdirectories of `root_dir` whose name contains `_`.
Each scene is expected to hold the module subdirectories listed under `directories`
(e.g. `camera/rgb_front`, `lidar`, `anno`, ...), with files named after zero-padded
frame indices.

The full set of visualizer names (matching the `visualizers/` directory):

| Visualizer name   | Module path (relative to scene) | Suffix     |
|-------------------|---------------------------------|------------|
| `json`            | `anno`                          | `.json.gz` |
| `image`           | `camera/rgb_front`              | `.jpg`     |
| `depth`           | `camera/depth_front`            | `.npz`     |
| `lidar`           | `lidar`                         | `.laz`     |
| `radar`           | `radar`                         | `.h5`      |
| `bev`             | `bev/birdview`                  | `.npz`     |
| `semantic`        | `camera/semantic_front`         | `.png`     |
| `lidar_semantic`  | `lidar_semantic`                | `.npz`     |
| `occ`             | `occ3d`                         | `.npz`     |

The module paths are examples only — adjust to your data layout. The template config
(`config/dataset_config.yaml`) enables a subset of these out of the box (`anno`,
`depth_image`, `bev_image`, `front_image`, `instance_image`, `lidar`, `radar`,
`bev_rendered`, `bev_masks`); the rest need a `directories` entry in your local config.
For instance, to visualize 3D occupancy:

```yaml
directories:
  occ:
    digits: 5
    path: occ3d          # adjust to your data layout
    suffix: .npz
    visualizer: occ
```

### Configuration reference

- `index` — how to enumerate valid frame indices. `path` + `suffix` point at the index
  manifest directory (default `anno/*.json.gz`); `digits` is only informational here,
  indices are any all-digit file names under that directory.
- `directories.<name>` — one entry per module:
  - `digits` — zero-padding width used to build the file name (e.g. `5` → `00042.jpg`).
  - `path` — path relative to the scene directory.
  - `suffix` — file extension.
  - `visualizer` — which visualizer to use (see table above).
  - Modules that don't exist on disk are simply skipped with a warning.
- `layout` — which modules appear in which column:

  ```yaml
  layout:
    left:
      - bev_image
      - depth_image
      - lidar
      - radar
    right:
      - front_image
      - anno
  ```

## Usage

```bash
python app.py
```

Then open <http://localhost:8080>.

- Use the **Scene / Index** dropdowns (or the prev / next buttons) to navigate.
- The column splitter is draggable and its position is remembered in `localStorage`.
- The `anno` (JSON) module has a **keyword filter** input. Patterns: `key` (exact),
  `key*` (prefix), `*key` (suffix), `*key*` (substring).
- Generated figures (depth colormaps, LiDAR/radar BEVs, ...) are written to a local
  `.cache/` directory and served from there.

## Notes

- Everything runs from the current working directory (`config/...`, `.cache/`), so
  launch the app from the `B2DVisualize/` folder.
- The old `templates/deprecated/` and `templates/overview.html` files are leftovers
  from an earlier QA-annotation tool and are no longer used by `app.py`.
