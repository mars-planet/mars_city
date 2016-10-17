=========================================================
Europa Installation.
=========================================================

:Author: Shridhar Mishra
:Date: 10 June 2015

These steps has to be followed in specific order for successful installation or its almost inevitable to get some weird java errors.

Prerequisites
******************
- JDK        ::

    sudo apt-get install openjdk-7-jdk
- ANT        ::

    sudo apt-get install ant
- Python     ::

    sudo apt-get install python
- subversion :: 

    sudo apt-get install subversion
- wget       :: 

    sudo apt-get install wget
- SWIG       :: 

    sudo apt-get install swig
- libantlr3c
- unzip      ::

    sudo apt-get install unzip

Now let us get the necessary packages to install libantlr3c.
::
    
    svn co http://europa-pso.googlecode.com/svn/ThirdParty/trunk plasma.ThirdParty

Get Europa.
-----------

::

    wget https://europa-pso.googlecode.com/files/europa-2.6-linux64.zip


cd ~/plasma.ThirdParty

Install ANTLR-C
------------------
::

    unzip libantlr3c-3.1.3.tar.bz2.
    cd plasma.ThirdParty/libantlr3c-3.1.3
    ./configure --enable-64bit ; make> sudo make install


**The above commands are for 64 bit machines.
for 32 bit machines remove --enable-64bit flag.**

Installing EUROPA.
------------------

::

    mkdir ~/europa
    cd ~/europaunzip ~/tmp/europa-2.1.2-linux.zip
    export EUROPA_HOME=~/europaexport
    LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EUROPA_HOME/lib


**Add the following lines to ~/.bashrc at the end.**
::

    EUROPA_HOME=~/europaLD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EUROPA_HOME/lib

Testing.
--------
::

    $EUROPA_HOME/bin/makeproject Light ~
    cd $EUROPA_HOME/examples/Light/
    ant

The Gui should appear for EUROPA.

If all the steps a correctly followed it should work.

Links.
------

`ANTLR-C installation <https://code.google.com/p/europa-pso/wiki/BuildingEuropa#Install>`_

`Europa Installation. <https://code.google.com/p/europa-pso/wiki/EuropaInstallation>`_

`Quick start <https://code.google.com/p/europa-pso/wiki/QuickStart>`_
