from xml.dom import minidom
import simplejson as json

from pymongo import MongoClient
from ast import literal_eval

from bson.objectid import ObjectId

def parse_element(element):
    dict_data = dict()
    if element.nodeType == element.TEXT_NODE:
        if element.data and element.data[0] != '\n':
            dict_data['data'] = element.data
    if element.nodeType not in [element.TEXT_NODE, element.DOCUMENT_NODE, 
                                element.DOCUMENT_TYPE_NODE]:
        for item in element.attributes.items():
            dict_data[item[0]] = item[1]
    if element.nodeType not in [element.TEXT_NODE, element.DOCUMENT_TYPE_NODE]:
        for child in element.childNodes:
            child_name, child_dict = parse_element(child)
            if child_name in dict_data:
                try:
                    dict_data[child_name].append(child_dict)
                except AttributeError:
                    dict_data[child_name] = [dict_data[child_name], child_dict]
            else:
                if child_dict:
                    dict_data[child_name] = child_dict 
    return element.nodeName, dict_data

def getClient():
    client = MongoClient('localhost',27017)
    db = client.PlexilDatabase
    return db.Plans

def addToCollection(filename):
    dom = minidom.parse(filename)
    element = parse_element(dom)
    if element[0] == '#document':
        element = json.dumps(element[1:], sort_keys=True, indent=4)
    else:
        element = json.dumps(element, sort_keys=True, indent=4)

    c = getClient()    
    data = literal_eval(element)
    data = data[0]
    return c.insert_one(data).inserted_id

def removeFromCollection(uid):
    c = getClient()
    c.remove({"_id":ObjectId(uid)})

if __name__ == '__main__':
    uid = addToCollection('/root/Desktop/plexil-x/examples/basic/ArrayComplex.plx')
    # removeFromCollection(uid)