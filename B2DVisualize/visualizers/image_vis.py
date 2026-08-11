from flask import request
import os
from util.webutil import get_proxy_url

class ImageVisualizer:
    def render(self, file_entry, cache_func):
        abs_path = file_entry["abs_path"]
        if not os.path.exists(abs_path):
            return f"<p style='color:red'>Missing image: {abs_path}</p>"

        cached_path = cache_func(file_entry["name"], abs_path)
        filename = os.path.basename(cached_path)
        url = get_proxy_url("cache", filename=filename)
        return f'<img src="{url}" style="max-width:100%;" />'