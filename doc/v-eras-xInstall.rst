v-eras-x machine Setup
=======================

.. highlightlang:: sh

This page documents how to install a V-ERAS station computer

SSH
---

Install ssh daemon::

  sudo apt-get install openssh-server
  sudo restart ssh

MISC
----
::

  sudo apt-get install vim-gnome
  sudo apt-get install g++

Oculus
------

Follow install procedure

xrandr
------

You can use ``xrandr`` to check and change the screen resolution::

  # check current setup (-d :0 is only necessary from remote)
  xrandr -d :0
  # + = preferred mode; * = selected mode
  # set 1920x1080 on HDMI2
  xrandr -d :0 --output HDMI2 --mode 1920x1080

Kinect
------

Follow install procedure
::
  sudo apt-get install python-visual

Tango
-----

Follow install procedure

Blender
-------
::

  sudo apt-get install blender


boot time processes
-------------------

To add an already available init.d process to boot time use::

  update-rc.d tango-starter defaults


nfs area setup
--------------

On v-eras-0::

  sudo apt-get install nfs-kernel-server

edit ``/etc/exports`` and add::

  /home    *(rw,sync,no_root_squash)

on the client machine::

  sudo apt-get install nfs-common
  sudo mkdir /nfshome
  sudo mount v-eras-0:/home /nfshome

then make a link to the virtual user remote home by::

  cd /home/virtual
  mv swarchive/ swarchive_local
  sudo ln -s /nfshome/virtual/swarchive swarchive

then edit ``/etc/fstab`` file to make it permanent by adding::

  v-eras-0:/home /nfshome nfs rsize=8192,wsize=8192,timeo=14,intr

usbip
-----

Server::

  # usbip is in this package, not in the usbip one
  sudo apt-get install linux-tools-common
  # start daemon, should give message saying to install some packages
  sudo usbipd -D
  # install them
  sudo apt-get install linux-tools-3.16.0-24-generic linux-cloud-tools-3.16.0-24-generic linux-tools-generic linux-cloud-tools-generic

  # do this before binding devices, or if binding fails
  sudo modprobe usbip-host
  # to check list of usb devices and see bus number use
  lsusb
  # to check list of local usb devices avalailable from usbip use
  sudo usbip list -l
  # to bind the device 8-4 to usbip use
  sudo usbip bind --busid=8-4
  # to see if the usbip daemon is running use
  sudo netstat -tap | grep usb

Client::

  # usbip is in this package, not in the usbip one
  sudo apt-get install linux-tools-common
  # install the same packages as above
  sudo apt-get install linux-tools-3.16.0-24-generic linux-cloud-tools-3.16.0-24-generic linux-tools-generic linux-cloud-tools-generic
  # load the vhci-hcd kernel module
  sudo modprobe vhci-hcd
  # check that the module is loaded:
  lsmod | grep vhci_hcd
  # check the usb devices exported by the remote host
  usbip list -r v-eras-0
  # connect to the remote usbip
  sudo usbip attach -r v-eras-0 -b 8-2
  # check if the remote usb device has been added
  lsusb

Troubleshooting
~~~~~~~~~~~~~~~

On the client, if you get::

  $ sudo usbip attach -r v-eras-0 -b 8-2
  libusbip: error: udev_device_new_from_subsystem_sysname failed
  usbip: error: open vhci_driver
  usbip: error: query

do::

  sudo modprobe vhci-hcd

If you get::

  $ usbip list -r v-eras-0
  usbip: info: no exportable devices found on v-eras-0

check that the server is up and running and that you bound the usb device.

See also http://www.howtoforge.com/how-to-set-up-a-usb-over-ip-server-and-client-with-ubuntu-10.04-p2
(instructions are outdated, but you can get an idea of the steps --
 the instructions on this page work for 14.10).
