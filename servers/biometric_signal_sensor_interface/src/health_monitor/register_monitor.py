from __future__ import absolute_import, division, print_function
import PyTango
import requests
import ConfigParser

__author__ = 'abhijith'

'''
Script to register the Biometric Monitor Tango Server
'''
def config_helper(section):
    '''
    Returns a dictonary of the configuration stored in
    ../biometric_monitor/config.cfg
        @param section: configuration section from the config file that,
                        has to be read
    '''
    dict_config = {}
    options = config.options(section)
    for option in options:
        try:
            dict_config[option] = config.get(section, option)
        except:
            print("exception on %s!" % option)
            dict_config[option] = None
    return dict_config

    
requests.packages.urllib3.disable_warnings()
config = ConfigParser.ConfigParser()
config.read("config.cfg")

dev_info = PyTango.DbDevInfo()
dev_info.server = config_helper("BiometricMonitor")['server_name']
dev_info._class = config_helper("BiometricMonitor")['class']
dev_info.name = config_helper("BiometricMonitor")['name']

print("Creating device: %s" % dev_info.name)
db = PyTango.Database()
db.add_device(dev_info)
