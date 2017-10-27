from device_proxy import DeviceProxy


class ConfigManager():
    def __init__(self, IP, port):
        pass

    def get_proxy(self, resource_path_URI):
        proxy = DeviceProxy(resource_path_URI)
        return proxy
