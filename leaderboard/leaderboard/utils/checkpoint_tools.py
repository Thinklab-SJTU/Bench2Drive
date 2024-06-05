import json
try:
    import simplejson as json
except ImportError:
    import json
import requests
import os.path


def autodetect_proxy():
    proxies = {}

    proxy_https = os.getenv('HTTPS_PROXY', os.getenv('https_proxy', None))
    proxy_http = os.getenv('HTTP_PROXY', os.getenv('http_proxy', None))

    if proxy_https:
        proxies['https'] = proxy_https
    if proxy_http:
        proxies['http'] = proxy_http

    return proxies


def fetch_dict(endpoint):
    data = None
    if endpoint.startswith(('http:', 'https:', 'ftp:')):
        proxies = autodetect_proxy()

        if proxies:
            response = requests.get(url=endpoint, proxies=proxies)
        else:
            response = requests.get(url=endpoint)

        try:
            data = response.json()
        except json.decoder.JSONDecodeError:
            data = {}
    else:
        data = {}
        if os.path.exists(endpoint):
            with open(endpoint) as fd:
                try:
                    data = json.load(fd)
                except json.JSONDecodeError:
                    data = {}

    return data


def save_dict(endpoint, data):
    if endpoint.startswith(('http:', 'https:', 'ftp:')):
        proxies = autodetect_proxy()

        if proxies:
            _ = requests.patch(url=endpoint, headers={'content-type':'application/json'}, data=json.dumps(data, indent=4, sort_keys=True), proxies=proxies)
        else:
            _ = requests.patch(url=endpoint, headers={'content-type':'application/json'}, data=json.dumps(data, indent=4, sort_keys=True))
    else:
        with open(endpoint, 'w') as fd:
            json.dump(data, fd, indent=4)
