# PlanActorsController - Tango Service

This service is used to store the information about the rovers and the astronauts (plan_actors).

The information is stored in a [sqlite3 database](https://docs.python.org/2/library/sqlite3.html).

The **Schema** of **plan_actors** is:
```
address - Primary key - text
type - text
avail_start - text
avail_end - text
capabilities - text
```

1) **address** : unique address associated with the plan_actor
2) **type** :  type of actor (rover/astronaut)
3) **avail_start** : availability start of the actor (Date - Time)
4) **avail_end** : availability end of the actor
5) **capabilities** : a JSON string
> The **avail_start** and **avail_end** fields should be in the format **YYYY-MM-DD HH:MM:SS**

### Project Structure:

```
/mars_city/MOCC/src/configurations/PlanActorsController
|-- docs
    `-- PlanActorsController.rst
    `-- index.rst
|-- src
    `-- PlanActorsController
    `-- DescendantHelper.py
|-- README.md

```

# Getting started

Refer [this](https://github.com/mars-planet/mars_city/blob/master/servers/body_tracker_v2/src/win/PyTango%20Setup/PyTango%20Installation%20Instructions/Instructions.txt) to set up Tango on your system.

# Usage
Register the PlanActorsController Service.

```
>>> import PyTango

>>> dev_info = PyTango.DbDevInfo()
>>> dev_info.server = "PlanActorsController/service"
>>> dev_info._class = "PlanActorsController"
>>> dev_info.name = "planActorsController/service/1"

>>> db = PyTango.Database()
>>> db.add_device(dev_info)
```

Finally start the PlanActorsController service.

```
$ python PowerSupplyDS.py test
Ready to accept request
```

You can use the service as follow:

```
>>> import PyTango

>>> plan_actor = PyTango.DeviceProxy("PlanActorsController/test/1")
>>> plan_actor.add_actor_meta(["address","rover","2017-05-05 12:45:00","2017-05-05 13:45:00","{string:capabilities}"])
'Actor added'
>>> plan_actor.get_actor_meta("address")
["address","rover","2017-05-05 12:45:00","2017-05-05 13:45:00","{string:capabilities}"]
```

For more  details regarding registering and starting device servers, refer [here](http://www.esrf.eu/computing/cs/tango/tango_doc/kernel_doc/pytango/latest/quicktour.html).



