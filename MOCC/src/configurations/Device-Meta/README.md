# Device Meta Data - Tango Service

This service returns all the meta data associated with a particular device.

Project Structure:

```
/mars_city/MOCC/src/configurations/Device-Meta
|-- docs
    `-- DeviceMetaData.rst
    `-- index.rst
|-- src
    `-- DeviceMetaData
    `-- DeviceMetaDataHelper.py
    `-- test_DeviceMetaData.py
|-- README.md

```

#Getting started

Refer [this](https://github.com/mars-planet/mars_city/blob/master/servers/body_tracker_v2/src/win/PyTango%20Setup/PyTango%20Installation%20Instructions/Instructions.txt) to set up Tango on your system.

#Usage
Consider *test/power_supply/1* is running on Tango. (Refer [this](http://www.esrf.eu/computing/cs/tango/tango_doc/kernel_doc/pytango/latest/quicktour.html) to get it running)
```
>>>import PyTango
>>>ser = PyTango.DeviceProxy("devicemetadata/test/1")
>>>ser.get_device_meta("test/power_supply/1")
>>>'{"attribute_types": "TANGO", "commands": [{"Init": {"arguments": {"Uninitialised": {"position": 1, "type": "DevVoid"}}, "return_object": {"Uninitialised": {"type": "DevVoid"}}}}, {"Ramp": {"arguments": {"Ramp tar
.
.
.
```
(Above output trimmed)

##Device Meta data schema

```
{
	"attribute_types":"TANGO",
	"commands":[
		{
			[command-names]:{
				"arguments":{
					"<arg-description>":{
						"position":<pos>,
						"type":<type>
					}
				},
				"return_object":{
					"<return-object-description>":{
						"type":<type>
					}
				}
			}
		},
		.
		.
		.
	],
	"attributes":[
		{
			"<attr-name>":{
				"writeable": True | False,
				"readable": True | False,
				"type": <type-no>
			}
		},
		.
		.
		.
	]
}
```

#For Testing

Register a new server with the Tango System. Refer [this](http://www.esrf.eu/computing/cs/tango/tango_doc/kernel_doc/pytango/latest/quicktour.html).
OR
Run this in a python shell
```
>>> import PyTango

>>> dev_info = PyTango.DbDevInfo()
>>> dev_info.server = "DeviceMetaData/test"
>>> dev_info._class = "DeviceMetaData"
>>> dev_info.name = "devicemetadata/test/1"

>>> db = PyTango.Database()
>>> db.add_device(dev_info)
```
Run the server on console with:
```
$ python DeviceMetaData test
Ready to accept request
```

Now run the unit test script: **test_DeviceMetaData.py**
```
----------------------------------------------------------------------
Ran 1 test in 0.006s

OK

```

