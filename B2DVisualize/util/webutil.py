from urllib.parse import urlparse
from flask import request, url_for

def get_proxy_url(endpoint, **values):
    """
    Generate an accessible URL based on the current request environment:
    - Automatically prepend proxy prefixes when running behind VSCode/Codespace proxy
    - Return a plain relative URL when running locally or without a proxy
    """
    # Generate a relative URL using Flask's url_for
    relative_url = url_for(endpoint, **values)

    # Flags and variables for proxy detection
    is_proxy = False
    prefix = ""

    # First, check for the X-Forwarded-Prefix header (common in proxy setups)
    if "X-Forwarded-Prefix" in request.headers:
        prefix = request.headers["X-Forwarded-Prefix"]
        is_proxy = True
    else:
        # Fallback: inspect the Referer header to see if it contains "/proxy/"
        referer = request.headers.get("Referer", "")
        if "/proxy/" in referer:
            parsed = urlparse(referer)
            path_parts = parsed.path.split("/proxy/")
            # path_parts[0]: prefix such as ws-xxxx/project-xxxx/user-xxxx
            # path_parts[1]: port number and the remaining path
            if len(path_parts) > 1:
                prefix = path_parts[0] + "/proxy/" + path_parts[1].split("/")[0]
            else:
                prefix = path_parts[0]
            is_proxy = True

    if is_proxy:
        # In proxy environments, prepend the detected prefix
        return prefix + relative_url
    else:
        # In non-proxy environments, return the relative URL directly
        return relative_url
