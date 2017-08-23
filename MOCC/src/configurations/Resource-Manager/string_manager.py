"""
Utility class to get python resource objects from strings and vice versa
"""

from models import ResourceObject


class StringManager:
    def str_to_object(self, resource_string):
        resource_data_list = resource_string.split("#")
        resource = ResourceObject(resource_data_list[0],
                                  resource_data_list[1],
                                  resource_data_list[2],
                                  resource_data_list[3],
                                  resource_data_list[4],
                                  resource_data_list[5])
        return resource
