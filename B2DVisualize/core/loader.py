import os
import glob

class DatasetLoader:
    def __init__(self, config):
        self.config = config
        self.root = config["dataset"]["root_dir"]
        self.index_cfg = config["index"]
        self.dir_cfg = config["directories"]

    # ---------- Scene ----------
    def list_scenes(self):
        if not os.path.exists(self.root):
            return []
        return sorted([
            d for d in os.listdir(self.root)
            if os.path.isdir(os.path.join(self.root, d)) and "_" in d
        ])

    # ---------- Index ----------
    def list_indices(self, scene):
        idx_cfg = self.index_cfg
        print(f"{scene}, {idx_cfg['path']}, *{idx_cfg['suffix']}")
        target_dir = os.path.join(
            self.root,
            scene,
            idx_cfg["path"]
        )

        pattern = os.path.join(target_dir, f"*{idx_cfg['suffix']}")
        files = glob.glob(pattern)

        indices = []
        for f in files:
            name = os.path.basename(f).replace(idx_cfg["suffix"], "")
            if name.isdigit():
                indices.append(int(name))

        return sorted(indices)

    def format_index(self, idx, digits):
        print(f"[debug] idx = {idx}, digits = {digits}, return = {str(idx).zfill(digits)}")
        return str(idx).zfill(digits)

    # ---------- Item ----------
    def get_item(self, scene, index_int):
        files = {}
        for name, cfg in self.dir_cfg.items():
            idx_str = self.format_index(index_int, cfg["digits"])
            abs_path = os.path.join(
                self.root,
                scene,
                cfg["path"],
                f"{idx_str}{cfg['suffix']}"
            )
            rel_path = os.path.relpath(abs_path, self.root)

            if os.path.exists(abs_path):
                files[name] = {
                    "name": name,
                    "path": abs_path,
                    "abs_path": abs_path,
                    "rel_path": rel_path,
                    "visualizer": cfg["visualizer"]
                }
            else:
                print(f"[WARN] missing file for module '{name}': {abs_path}")
        return files

