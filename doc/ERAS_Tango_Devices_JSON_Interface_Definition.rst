.. raw:: html

    <style>
    .red {color:red}
    .green {color:green}
    .blue {color:blue}
    </style>
    
.. role:: red
.. role:: green
.. role:: blue
   
ERAS' Tango Devices' JSON Interface Definition
==============================================

Abstract
--------

The purpose of this document is to specify the common methods required of all
Tango device servers in the ERAS network.
The first section presents the describe_attributes method, which is used to
obtain a device's attributes' meta-information.
The second section treats the describe_commands method, used to obtain a
device's methods' meta-information.
For the remaining of the document, :green:`string`, :green:`int` and
:green:`bool` stand for their respective Python types, whereas
:green:`TangoType` stands for any of the types accepted as a attribute's type
by Tango and :green:`Json` stands for a special type: a JSON-formatted string.
Moreover, :red:`[` A|...|Z :red:`]` indicates a choice among predefined
values.


The *describe_attributes* method
-------------------------------

Signature:

.. code:: python

    string describe_attributes(cls)

Format of the string returned:

.. code:: python

    {
    "attribute_types": ["TANGO"|"JSON"|"MIXED"],
    "attributes": {
                 "[attribute_name_1]": {
                                       "type": [TangoType|"Json"],
                                       ["schema": string,]
                                       ["readable": bool,]
                                       ["writeable": bool,]
                                       },
                 "[attribute_name_2]": ...
                 ...
                 "[attribute_name_N]": ...
                 }
    }

The meaning of each key in the schema is described below:

* **"attribute_types"**: should be "TANGO", if all attributes are of a
  :green:`TangoType` type, "JSON" if all attributes are of type :green:`Json`,
  and "MIXED" otherwise.
* **"attributes"**: contains a dictionary with the description of all the
  device's attributes. The attribute's name is used as key. 
* **"[attribute_name_#]"**: it should be substituted with the particular
  attribute's name. All attributes available in a device should be present.
  Contains a dictionary describing the attribute.
* **"type"**: contains the type of the attribute's value.
* **"schema"**: contains the schema for the value of a attribute of type
  :green:`Json`. Required if the attribute is of type :green:`Json` and
  forbidden otherwise.
* **"readable"**: describes whether the current attribute's value can be read.
  Optional element with default :green:`True`.
* **"writeable"**: describes whether the attribute's value can be written.
  Optional element with default :green:`False`.

    
The *describe_commands* method
------------------------------

Signature:

.. code:: python

    string describe_commands(cls)

Format of the string returned:

.. code:: python

    {
    "command_types": ["TANGO"|"JSON"|"MIXED"],
    "commands": {
                "[command_name_1]": {
                                    "arguments": {
                                                 "[argument_name_1]": {
                                                                      "position": int,
                                                                      "type": [TangoType|"Json"],
                                                                      ["schema": string,]
                                                                      },
                                                 "[argument_name_2]": ...
                                                 ...
                                                 "[argument_name_M]": ...
                                                 },
                                    "return_object": {
                                                     "type": [TangoType|"Json"],
                                                     ["schema": string,]
                                                     }
                                    },
                "[command_name_2]": ...
                ...
                "[command_name_N]": ...
                }
    }

The meaning of each key in the schema is described below:

* **"command_types"**: should be "TANGO", if all commands' parameters and
  return objects are of a :green:`TangoType` type, "JSON" if all commands'
  parameters and return objects are of type :green:`Json`, and "MIXED"
  otherwise.
* **"commands"**: contains a dictionary with the description of all the
  device's commands. The command's name is used as key.
* **"[command_name_#]"**: it should be substituted with the particular
  command's name. All commands available in a device should be present.
* **"arguments"**: contains a dictionary of all arguments of a particular
  command. The argument's name is used as key.
* **"return_object"**: contains a dictionary describing the return object.
* **"[argument_name_#]"**: it should be substituted with the particular
  argument's name. All command's arguments should be present. Contains a
  dictionary describing the argument.
* **"position"**: contains the position of the argument in the command's
  arguments list (1-indexed, to allow for the :green:`self` argument).
* **"type"**: contains the type of the argument's or return object's value.
* **"schema"**: contains the schema for the value of a argument or return
  object of type :green:`Json`. Required if the attribute is of type
  :green:`Json` and forbidden otherwise.
