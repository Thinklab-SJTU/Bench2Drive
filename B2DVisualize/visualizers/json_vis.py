import json, gzip, html
from .base import BaseVisualizer
import json, gzip, html
from .base import BaseVisualizer

class JsonVisualizer:
    def render(self, file_entry, cache_func, initial_filter=""):
        """
        Render a JSON file as modular HTML.
        Only filtering functionality is preserved.
        initial_filter: string to initialize the filter input
        """
        # Cache the original file
        cache_func(file_entry["name"], file_entry["abs_path"])
        abs_path = file_entry["abs_path"]

        # Load JSON data (support both .json and .json.gz)
        if abs_path.endswith(".gz"):
            with gzip.open(abs_path, "rt", encoding="utf-8") as f:
                data = json.load(f)
        else:
            with open(abs_path, "r", encoding="utf-8") as f:
                data = json.load(f)

        # Add list indices to dictionaries inside lists (for easier identification)
        data = append_list_id(data)

        # Top control bar with a keyword filter input
        control_html = f"""
        <div style="margin-bottom:5px; display:flex; gap:10px; align-items:center;">
            <input type="text"
                id="filter_{file_entry['name']}"
                value="{html.escape(initial_filter)}"
                placeholder="Filter keywords">
            <button onclick="applyFilter('{file_entry['name']}')">Filter</button>
        </div>
        """

        # Render JSON content directly using <pre>
        json_html = f"<pre>{html.escape(json.dumps(data, indent=2))}</pre>"

        # Final HTML block for this JSON module
        module_html = f"""
        <div id='module_{file_entry['name']}' style='max-height:500px; overflow:auto; border:1px solid #ccc; padding:5px; margin-bottom:10px;'>
            {control_html}
            <div id='content_{file_entry['name']}'>{json_html}</div>
        </div>
        """
        return module_html

def append_list_id(obj):
    """
    Recursively traverse a JSON-like object and, for each dictionary
    inside a list, add a 'dict_id' field indicating its index.

    This is useful for debugging, visualization, or stable identification
    of list elements.
    """
    if isinstance(obj, dict):
        return {k: append_list_id(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        result = []
        for i, item in enumerate(obj):
            if isinstance(item, dict):
                item['dict_id'] = i
            result.append(append_list_id(item))
        return result
    else:
        return obj


def filter_json_tree(json_data, filter_keywords):
    """
    Filter a JSON tree based on a list of keyword patterns.

    Supported patterns:
    - exact match:      "key"
    - prefix match:     "key*"
    - suffix match:     "*key"
    - substring match:  "*key*"

    Returns a pruned JSON structure where only matching branches are kept.
    """
    if not filter_keywords:
        return json_data

    def matches_keyword(value, keyword):
        # Match value against a single keyword pattern
        if keyword.startswith("*") and keyword.endswith("*"):
            return keyword[1:-1] in value
        elif keyword.startswith("*"):
            return value.endswith(keyword[1:])
        elif keyword.endswith("*"):
            return value.startswith(keyword[:-1])
        else:
            return value == keyword

    def matches_any_keyword(value):
        # Check if the value matches any of the provided keywords
        return any(matches_keyword(value, k) for k in filter_keywords)

    def recursive_filter(obj):
        # Recursively filter dictionaries and lists
        if isinstance(obj, dict):
            result = {}
            for k, v in obj.items():
                filtered_v = recursive_filter(v)
                # Keep this key if the key itself matches or any child matches
                if matches_any_keyword(k) or filtered_v is not None:
                    result[k] = filtered_v if filtered_v is not None else v
            return result if result else None
        elif isinstance(obj, list):
            filtered_list = [recursive_filter(i) for i in obj]
            filtered_list = [i for i in filtered_list if i is not None]
            return filtered_list if filtered_list else None
        else:
            # Leaf node: keep only if it matches a keyword
            return obj if matches_any_keyword(str(obj)) else None

    return recursive_filter(json_data)
