import os, shutil

CACHE_DIR = ".cache"
os.makedirs(CACHE_DIR, exist_ok=True)

def cache_file(module_name, abs_path):
    """
    copy file to .cache/module_name + suffix
    """
    _, ext = os.path.splitext(abs_path)
    cached_name = f"{module_name}{ext}"  # use module name + file suffix
    cached_path = os.path.join(CACHE_DIR, cached_name)

    shutil.copy2(abs_path, cached_path)
    return cached_path