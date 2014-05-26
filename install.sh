#!/bin/bash
#To run this shell script give "sh install.sh"

echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "Setting up ERAS Project"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
sleep 3

echo "--------------------------------------------------------------------"
echo "Updating all the packages"
echo "--------------------------------------------------------------------"
sleep 1
sudo apt-get --yes update
echo "Done Updating"

echo "--------------------------------------------------------------------"
echo "Installing MySql"
echo "--------------------------------------------------------------------"
sleep 1
sudo apt-get install --yes mysql-server mysql-client
sudo apt-get install --yes python-mysqldb
echo "Done installing MySql"

echo "--------------------------------------------------------------------"
echo "Installation of g++ and javac"
echo "--------------------------------------------------------------------"
sleep 1
sudo apt-get install --yes g++
sudo apt-get install --yes default-jdk
echo "Done installing g++ and javac"

echo "--------------------------------------------------------------------"
echo "Installing ssh daemon"
echo "--------------------------------------------------------------------"
sleep 1
sudo apt-get install --yes openssh-server
sudo restart ssh
echo "done installing ssh daemon"

echo "--------------------------------------------------------------------"
echo "Tango Installation"
echo "--------------------------------------------------------------------"
echo "Installing dependencies"
sleep 1
echo "The current PyTango version has three major dependencies:"
echo "Boost-python"
sudo apt-get install --yes libboost-python-dev
echo "Numpy installation"
sudo apt-get install --yes python-numpy
echo "IPython Installation"
sudo apt-get install --yes ipython

echo "--------------------------------------------------------------------"
echo "Installing PyTango"
echo "--------------------------------------------------------------------"
sleep 1
sudo apt-get install --yes tango-common tango-db tango-starter tango-test python-pytango libtango7-doc libtango7-dev tango-starter
echo "Done Installing PyTango"

echo "--------------------------------------------------------------------"
echo "Installing redis"
echo "--------------------------------------------------------------------"
sudo apt-get install --yes python-redis
echo "Done installing redis"

echo "--------------------------------------------------------------------"
echo "Installing Matplotlib neccessary for ploting"
echo "--------------------------------------------------------------------"
sudo apt-get install --yes python-matplotlib
echo "Done installing Matplotlib"

echo "--------------------------------------------------------------------"
echo "Installing SciPy"
echo "--------------------------------------------------------------------"
sudo apt-get install --yes python-redis
echo "Done installing SciPy"

echo "--------------------------------------------------------------------"
echo "Installing NetworkX"
echo "--------------------------------------------------------------------"
sudo apt-get install --yes python-redis
echo "Done installing NetworkX"

echo "--------------------------------------------------------------------"
echo "Installing ffnet"
echo "--------------------------------------------------------------------"
sudo apt-get install --yes python-setuptools
sudo easy_install ffnet
echo "Done installing ffnet"
echo ""
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "Successfully installed all the neccessary packages of ERAS Project"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"