# Device Meta Data - Tango Service

Provided a device id (possibly partial), this service returns all the devices that are associated with it.

Project Structure:

```
/mars_city/MOCC/src/configurations/Get-Descendant
|-- docs
    `-- GetDescendant.rst
    `-- index.rst
|-- src
    `-- GetDescendant
    `-- DescendantHelper.py
|-- README.md

```

#Getting started

Refer [this](https://github.com/mars-planet/mars_city/blob/master/servers/body_tracker_v2/src/win/PyTango%20Setup/PyTango%20Installation%20Instructions/Instructions.txt) to set up Tango on your system.

#Usage
Consider *test/power_supply/1*, *test/power_supply/2*, *test/water_supply/1* and *test/water_supply/2* is running on Tango. (Refer [this](http://www.esrf.eu/computing/cs/tango/tango_doc/kernel_doc/pytango/latest/quicktour.html) to get it running)
Also start the GetDescendant device server, say, with the id "test/getdescendant/1"
```
>>>import PyTango
>>>ser = PyTango.DeviceProxy("test/getdescendant/")
>>>ser.get_descendant("test/power_supply")
>>>['test/power_supply/1', 'test/power_supply/2']
```


##To enable more levels of hierarchy than what Tango provides out of the box (3 level - *domain/family/member*)

Tango provides the domain/family/member schema for the device servers, thus limiting our hierarchy of devices to upto three layers.

To have more levels of hierarchy, a **standard** is to be maintained.

The first two levels of the Tango can be reserved for the Top-Level Stuff and third level can be everything related to mocc.
Everything device below that can be separated using a meta-character, such as  "**:**" (colon).

This allows any number of levels in the hierarchy.

For example: 
	- mars/ground/mocc:telemetry:store
	- mars/ground/rovers:rovers1
	- mars/orbit/sat1:antenna1
