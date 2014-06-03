#!/bin/bash
# File containing all modification required to run the Gazebo 

# Version 1
# Author Kunal Tyagi

# Not required to be run as root
# Insert notification message, if required

# ERAS directory containing the .hg folder
PWD=`pwd`
ERAS_DIR=$PWD/../../
ROOT_UID=0     # Only users with $UID 0 have root privileges.
E_ROOT=85      # Root exit error
E_ARG=86       # Unnecessary arguments provided
GAZEBO_INSTALLED=`which gazebo | wc -l`

# if run as root, echo a warning and exit. Change the behaviour if required
if [ "$UID" = "$ROOT_ID" ]
then
    echo "Being a root is not required. Continuing ... "
    exit $E_ROOT
fi

# test is come command line argument is given
if [ -n "$1" ]
then
    echo "Usage: source `basename $0`"
    # `basename $0` is the script's filename
    echo "PS: 'source' is required. But no arguments required"
    exit $E_ARG
fi

if [ -n "$GAZEBO_INSTALLED" ]
then
    echo "Setting up CLI environment" 

    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$ERAS_DIR/servers/gazebo/models/

    # required soon: a plugin path, somewhere the lib*.so files would reside
    # NB: the lib*.so files are made from C++ files and required for Gazebo to run properly
    # export GAZEBO_PLUGIN_PATH=$(pwd)/lib
else
    echo "No installation of Gazebo found"
    echo "Install Gazebo version 2.2 from http://gazebosim.org/wiki/2.2/install"
fi
