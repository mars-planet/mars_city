#!/usr/bin/env python

from __future__ import division, print_function
import json


def get_meta(dev):

    attribute_list = dev.get_attribute_list()
    command_list = dev.command_list_query()

    attributes = []

    for attr in attribute_list:
        attrname = {}
        W = False
        a = dev.attribute_query(attr)
        if "read" in str(a.writable).lower():
            if "write" in str(a.writable).lower():
                R = True
                W = True
            elif "read" in str(a.writable).lower():
                R = True
        atr = {"type": str(a.data_type), "readable": R, "writeable": W}
        attrname[a.name] = atr
        attributes.append(attrname)

    commands = []

    for a in command_list:
        cmdname = {}
        arg = {}
        aa = {}
        aa[a.in_type_desc] = {"position": 1, "type": str(a.in_type)}
        arg["arguments"] = aa
        ret = {}
        ret[a.out_type_desc] = {"type": str(a.out_type)}
        arg["return_object"] = ret
        cmdname[a.cmd_name] = arg
        commands.append(cmdname)

    command_json = {"attribute_types": "TANGO",
                    "attributes": attributes, "commands": commands}

    return json.dumps(command_json)
