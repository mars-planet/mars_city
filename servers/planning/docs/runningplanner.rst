=========================================================
Steps to run servers.
=========================================================

:Author: Shridhar Mishra
:Date: 17 September 2015

These are the steps that should be followed to run all the required
servers after completing all the necessary installations to use Europa
along with the Ros driven husky rover.

Prerequisites
******************
- `Europa Installed
  <https://eras.readthedocs.org/en/latest/servers/planning/docs/index
  .html>`_
- Updated planning repo from eras.

Steps to be followed.
^^^^^^^^^^^^^^^^^^^^^

::

    cd ~/eras/server/planning/src/
    python pso 1
    cd ./simple_example
    python create_server.py
    python Publisher.py test
    python get_coord.py

Note:
"""""
- **create_server.py** - creates new server and registers tango device.
  It is required only for the first time.
- **get_cord.py** - Returns the coordinates for the husky rover.

