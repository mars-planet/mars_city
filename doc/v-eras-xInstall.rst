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


boot time processes
-------------------
To add an already available init.d process to boot time use
update-rc.d tango-starter defaults


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
cd /home/virtual
mv swarchive/ swarchive_local
sudo ln -s /nfshome/virtual/swarchive swarchive

then edit /etc/fstab file to make it permanent by adding:
v-eras-0:/home /nfshome nfs rsize=8192,wsize=8192,timeo=14,intr

usbip
-----

Server:

  sudo apt-get install linux-tools-common  # usbip is in this package, not in the usbip one

  sudo usbipd -D  # start daemon, should give message saying to install the followings

  sudo apt-get install linux-tools-3.16.0-24-generic linux-cloud-tools-3.16.0-24-generic linux-tools-generic linux-cloud-tools-generic  # install them

  lsusb  # to check list of usb devices and see bus number

  sudo usbip list -l  # to check list of usb devices avalailable from usbip

  sudo usbip bind --busid=8-4  # to bind the device 8-4 to usbip

  sudo netstat -tap | grep usb  # to see if the usbip daemon is running

Client:

   sudo apt-get install linux-tools-common  # usbip is in this package, not in the usbip one

  sudo apt-get install linux-tools-3.16.0-24-generic linux-cloud-tools-3.16.0-24-generic linux-tools-generic linux-cloud-tools-generic  # install the same packages as above

  sudo usbip attach -r serverip -b 8-4

# maybe some modprob is needed too?
# see also http://www.howtoforge.com/how-to-set-up-a-usb-over-ip-server-and-client-with-ubuntu-10.04-p2
