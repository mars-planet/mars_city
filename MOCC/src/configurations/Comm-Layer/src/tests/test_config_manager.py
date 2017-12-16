import unittest
import json
import tempfile
import sys
sys.path.insert(0, '..')
import config_manager

class ConfigManagerTest(unittest.TestCase):
	def setUp(self):
		config_manager.app.testing = True
		self.app = config_manager.app.test_client()

	def test_save_path(self):
		print("Testing /save")

		response = self.app.get('/save/test_config_manager_1', follow_redirects=True)
		assert b'Successfully saved device address' in response.data

	def test_get_addr_path(self):
		print("Testing /get_addr")
		
		# Localhost
		ip_addr = "127.0.0.1"
		# ConfigManagerTest file name
		tango_addr = "test_config_manager"
		self.app.get('/save/test_config_manager_1', follow_redirects=True)
		response = self.app.get('/get_addr/test_config_manager_1', follow_redirects=True)
		response_json = json.loads(response.data) 
		response_json = response_json[0]
		assert ip_addr in response_json['ip_addr']
		assert tango_addr in response_json['tango_addr']
		
	
	
if __name__ == '__main__':
	unittest.main()
