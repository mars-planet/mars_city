from config_manager import ConfigManager

if __name__ == '__main__':

    # Connecting to ConfigManager
    cf = ConfigManager('192.168.0.3', '8080')

    # getting access to the health monitor proxy
    health_monitor_proxy = cf.get_proxy('/health_monitor')

    print('Testing connection with health monitor:')
    print(health_monitor_proxy.test_connection(), '\n')

    functions = health_monitor_proxy.get_function_list()
    functions = functions['functions']

    print('The available functions are')
    for function in functions:
        print(' -', function)
    print()

    attributes = health_monitor_proxy.get_attribute_list()
    attributes = attributes['attributes']

    print('The available attributes are')
    for attribute in attributes:
        print(' -', attribute)
    print()

    # Executing commands on the health monitor
    print('Executing : administer_insulin')
    print(health_monitor_proxy.administer_insulin(), '\n')

    print('Executing : ressucitate')
    print(health_monitor_proxy.ressucitate())
