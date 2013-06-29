Tango Setup
===========

.. highlightlang:: sh

This page documents how to install Tango on Ubuntu.  These instructions
have been tested on Ubuntu 13.04.

MySQL
-----

First you have to install MySQL::

  sudo apt-get install mysql-server mysql-client

During the installation of **mysql**, you can leave the root password empty.
If you specify a password you will have to enter it later, while installing
**tango-db**.


Tango
-----

After installing MySQL you can install tango::

  sudo apt-get install tango-common tango-db tango-starter tango-test python-pytango libtango7-doc libtango7-dev

During the installation of **tango-common**, enter ``pcname:10000`` as
TANGO host, where *pcname* is the name of your machine.

During the installation **tango-db** select:

* Configure database for tango-db with dbconfig-common? [yes]
* Password of the database's administrative user: [leave empty]
* MySQL application password for tango-db: [leave empty]

Next you have to install **libtango-java**.  If this is not available in the
repositories, you can add the launchpad repository manually::

    sudo add-apt-repository 'deb http://ppa.launchpad.net/tango-controls/core/ubuntu precise main'
    sudo apt-get update
    sudo apt-get install libtango-java


.. note::
    Currently the repository doesn't have packages for *quantal*/*raring*,
    so it is necessary to specify *precise* even if you are running a more
    recent version.
    If you use ``sudo add-apt-repository ppa:tango-controls/core``, your
    version of Ubuntu will be selected, and you will have to change it to
    *precise* manually.

You should now be able to lauch **astor** and **jive** from the terminal.

For more information see the Troubleshooting section below and this link:
http://www.tango-controls.org/howtos/binary_deb


Troubleshooting
---------------

At the end of the installation, you should have two tango processes running::

    $ ps aux | grep tango
    tango    15451  1.0  0.1  71800  9820 ?        Sl   10:56   0:00 /usr/lib/tango/Starter pcname
    tango    21109  0.0  0.1  94396 10752 ?        Sl   May18   0:20 /usr/lib/tango/DataBaseds 2 -ORBendPoint giop:tcp::10000

The first one is from **tango-starter**, the second one from **tango-db**.

If you don't see them, you can try to reinstall these two packages, using::

    sudo apt-get remove package-name
    sudo apt-get install package-name

If you are reinstalling **tango-db** and/or if you get this error::

    An error occurred while installing the database:
    ERROR 2002 (HY000): Can't connect to local MySQL server through socket
    '/var/run/mysqld/mysqld.sock' (2)

you have to select <Yes> when asked "Deconfigure database for tango-db with
*dbconfig-common*?".

If you are still having problem you can try the following things:

* check that ``/etc/tangorc`` contains ``TANGO_HOST=pcname:10000``;
* try to set the environment variable ``TANGO_HOST``.  You can also add
  the following line to your ``~/.bashrc`` to make it automatic (you will have
  to restart bash)::

    export TANGO_HOST=pcname:10000

* try to (re)start **tango-db** and/or **tango-starter** manually using::

    sudo /etc/init.d/tango-db start
    sudo /etc/init.d/tango-starter start

Adding a new server in Tango
----------------------------
To register a new server run **jive**, select ``Edit -> Create Server`` and provide:

* the executable name and the instance name (ex: legorcx/c1b8)
* the Class name 
* the device name in the format: ``C3/subsystem/device``

Then start the java/python/C++ application always providing the instance name, example::

  python legorcx c1b8

and the Class properties will be automatically filled in the database


