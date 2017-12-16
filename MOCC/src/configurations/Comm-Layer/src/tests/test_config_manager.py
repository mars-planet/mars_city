import unittest
import requests
import json


class ConfigManagerTest(unittest.TestCase):

    def test_get_addr_path(self):
		print("Testing /get_addr")
		
		# Localhost
		ip_addr = "127.0.0.1"
		# ConfigManagerTest file name
		tango_addr = "test_config_manager"

		response = requests.get('http://localhost:5000/get_addr/test_config_manager/')
		response_json = json.loads(response.text) 
		response_json = response_json[0]
		self.assertEqual(ip_addr, response_json['ip_addr'])
		self.assertEqual(tango_addr, response_json['tango_addr'])
		

    def test_save_path(self):
    	print("Testing /save")

        response = requests.get('http://localhost:5000/save/test_config_manager/')
        self.assertEqual("Successfully saved device address", response.text)

	
if __name__ == '__main__':
	suite = unittest.TestLoader().loadTestsFromTestCase(ConfigManagerTest)
	unittest.TextTestRunner(verbosity=2).run(suite)
