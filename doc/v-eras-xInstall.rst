v-eras-x machine Setup
=======================

.. highlightlang:: sh

This page documents how to install a V-ERAS station computer

SSH
-----
- Install ssh daemon
  sudo apt-get install openssh-server
  sudo restart ssh

MISC
-----
sudo apt-get install vim-gnome
sudo apt-get install g++

Oculus
------
Follow install procedure

Kinect
------
Follow install procedure
sudo apt-get install python-visual

Tango
-----
Follow install procedure

Blender
-------
sudo apt-get install blender 


nfs area setup
--------------
on v-eras-0
sudo apt-get install nfs-kernel-server
edit  /etc/exports and add:
/home    *(rw,sync,no_root_squash)

on the client machine:
sudo apt-get install nfs-common
sudo mkdir /nfshome
sudo mount v-eras-0:/home /nfshome

then make a link to the virtual user remote home by:
cd /home
sudo mv virtual virtual_local
sudo ln -s /nfshome/virtual virtual

then edit /etc/fstab file to make it permanent by adding:
v-eras-0:/home /nfshome nfs rsize=8192,wsize=8192,timeo=14,intr





