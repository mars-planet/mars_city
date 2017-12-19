import unittest
import os
import json
import sys
sys.path.insert(0, '..')
import config_manager
import tango_db_helper as DBHelper

class ConfigManagerTest(unittest.TestCase):

	def setUp(self):
		"""Method to set up the test app before every test."""
		config_manager.app.testing = True
		self.app = config_manager.app.test_client()
		
	def test_save_path(self):
		"""Calls the /save API to test saving of device address."""
		print("Testing /save")

		response = self.app.get('/save/test_config_manager', follow_redirects=True)
		assert b'Successfully saved device address' in response.data

	def test_insert_path(self):
		"""Calls the /insert API to test inserting of device address."""
		print("Testing /insert")

		response = self.app.get('/insert/test_config_manager_insert', follow_redirects=True)
		assert b'Successfully inserted device address' in response.data

	def test_insert_path_fail(self):
		"""Calls the /insert API to test failure of inserting of device address."""
		print("Testing /insert")

		response = self.app.get('/insert/test_config_manager_insert', follow_redirects=True)
		assert b'Device address exist' in response.data

	def test_update_path(self):
		"""Calls the /update API to test updating of device address."""
		print("Testing /update")

		response = self.app.get('/update/test_config_manager', follow_redirects=True)
		assert b'Successfully updated device address' in response.data

	def test_update_path_fail(self):
		"""Calls the /insert API to test inserting of device address."""
		print("Testing /update")

		response = self.app.get('/update/test_config_manager_update', follow_redirects=True)
		assert b'Device address doesn\'t exist' in response.data

	def test_get_addr_path(self):
		"""Calls the /get_addr API to test retrieval of device address."""
		print("Testing /get_addr")
		
		# Localhost
		ip_addr = "127.0.0.1"
		# ConfigManagerTest file name
		tango_addr = "test_config_manager"
		self.app.get('/save/test_config_manager', follow_redirects=True)
		response = self.app.get('/get_addr/test_config_manager', follow_redirects=True)
		response_json = json.loads(response.data) 
		response_json = response_json[0]
		assert ip_addr in response_json['ip_addr']
		assert tango_addr in response_json['tango_addr']
		
	def test_get_addr_path_fail(self):
		"""Calls the /get_addr API to test the failur of retrieval of device address."""
		print("Testing /get_addr with dummy data not in database")
		
		# Localhost
		ip_addr = "127.0.0.1"
		# ConfigManagerTest file name
		tango_addr = "test_config_manager"
		response = self.app.get('/get_addr/test_config_manager_dummy', follow_redirects=True)
		assert "Does not exist" in response.data
	
	
if __name__ == '__main__':
	DBHelper.create()
	unittest.main(exit=False)
	DBHelper.delete()
