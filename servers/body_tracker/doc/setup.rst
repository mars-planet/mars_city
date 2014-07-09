=============================================================================
Instructions for setting up environment for running Body Tracker application
=============================================================================

Installing OpenNI2, libfreenect and the Kinect sensor on Ubuntu 14.04 (64-bit)
------------------------------------------------------------------------------

The manual process for installing OpenNI2, libfreenect and Kinect sensor on
Linux is quite tricky. Hence a setup script is provided to ease the
installation process.

.. note::

    * openni.org referenced in some documentation is not online anymore.
    * You need to be running a 64-bit OS.
    * Commands are for Ubuntu 14.04 LTS or similar Debian based system.
    * The script has been tested with Ubuntu 14.04.

Quick install with `kinect-install.sh`
......................................

It is provided inside **body_tracker/tracker** at ERAS code base on github.
This script automates all the manual steps provided in the next section. Note
that it needs to be run with sudo. This ensures that also the non-root account
gets all the required permissions.

.. note::

    * Run commands as root.
    * The script installs Oracle Java 8. If you already have javac, you can
      comment out the `install-java8` line


1. Make sure you're connected to internet and start a terminal
2. Run::

        sudo ./kinect-install.sh

3. Wait a while for everything to install and answer **Yes** when the Java
   installer asks you to accept the license, no other interaction is needed.

Move to section - `Installing NiTE2 - A middleware library that calculates
skeleton` directly.

Manual install
..............

**Get prerequisites:**

1.  Install packages
    ::

        sudo apt-get install git-core g++ cmake libudev-dev libxi-dev libxmu-dev python libusb-1.0-0-dev libudev-dev freeglut3-dev doxygen graphviz

2. Get OpenNI2 library from https://github.com/OpenNI/OpenNI2
   ::

        git clone https://github.com/OpenNI/OpenNI2.git
        cd OpenNI2
        #Save path for further reference
        OPENNI_DIR="${PWD}"

3. Fix some issues with compiling (in OpenNI2 commit 7bef8f639e4d64a85a794e85fe3049dbb2acd32e)

   **Comment out "treat warnings as errors" option in PSCommon Makefile**
   ::

        sed -i '/-Werror/ s/^/#/' ${OPENNI_DIR}/ThirdParty/PSCommon/BuildSystem/CommonCppMakefile

   **Fix NiViewer Makefile (missing -lpthread in linker arguments)**
   ::

        echo "LDFLAGS += -lpthread" >> ${OPENNI_DIR}/Source/Tools/NiViewer/Makefile
        make

4. Add udev rules to access the sensor as normal user

   **Add udev rules for Primesense sensor device IDs**
   ::

        sudo ${OPENNI_DIR}/Packaging/Linux/install.sh

   - libfreenect ships with 51-kinect.rules file, but it doesn't seem to work
     with Ubuntu 14.04.
   - The following rules allow video group to access the Kinect components(
     Motor, Audio, Camera). Save in /etc/udev/rules.d/
   - **eg. /etc/udev/rules.d/51-kinect.rules**

   **Rules for Kinect & Kinect for Windows' Motor, Audio and Camera**
   ::

        SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE:="0666", OWNER:="root", GROUP:="video"
        SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE:="0666", OWNER:="root", GROUP:="video"
        SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE:="0666", OWNER:="root", GROUP:="video"
        SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE:="0666", OWNER:="root", GROUP:="video"
        SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE:="0666", OWNER:="root", GROUP:="video"
        SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE:="0666", OWNER:="root", GROUP:="video"

5. Join user to group

   **Add current user to video group (specified in udev rules)**
   ::

        sudo gpasswd -a ${USER} video

6. Install Oracle java8 (optional, but compiling OpenNI2 Java wrappers requires javac):
   ::

        sudo add-apt-repository ppa:webupd8team/java
        sudo apt-get update
        sudo apt-get install oracle-java8-installer

7. For Kinect support (Freenect bridge driver for Kinect to work under Linux/OSX):
    - https://github.com/OpenKinect/libfreenect/tree/master/OpenNI2-FreenectDriver
    - compile and copy driver to OpenNI2 directory:

   ::

        git clone https://github.com/OpenKinect/libfreenect.git
        cd libfreenect
        mkdir build; cd build
        cmake .. -DBUILD_OPENNI2_DRIVER=ON
        make
        cp -L lib/OpenNI2-FreenectDriver/libFreenectDriver.so ${OPENNI_DIR}/Bin/x64-Release/OpenNI2/Drivers/

8. If all went okay, you should now be able to plug in a Kinect / Xtion /
   Primesense sensor and run the example programs:
   ::

        cd ${OPENNI_DIR}/Bin/x64-Release/
        ./NiViewer

* **Press '?' in NiViewer to get help and try out the various modes.**
* **To start recording sensor data into a file, press 'c'. To stop recording,
  press 'x'. A .oni file is generated in the working directory.**

Troubleshooting
...............

* **Installation failed and I can't remove the leftovers**
    * Use sudo to remove the files - they were copied there as root
* **The sensor is not found**
    * Try unplugging the sensor and plugging it back again, then wait for few
      seconds (10 to be sure).
    * Check last lines of `dmesg` output to see if the device shows up
    * Try running the program with sudo
        * If it works then, make sure you've either logged out and back in
          again after you added your user to the `video` group, or type
          `newgrp video` and see if you can then run it as normal user.
* **Something fails with the compile**
    * The script makes some fixes to build files and these may be very specific
      to a certain commit. If you uncomment the lines with `NICOMMIT=...` and
      `FREENECTCOMMIT=...` you can specify which version to checkout for the
      builds.
    * In `kinect-install.sh`:

      ::

            # uncomment to check out the versions this script was written for
            # NICOMMIT="7bef8f639e4d64a85a794e85fe3049dbb2acd32e"
            # FREENECTCOMMIT="cb0254a10dbeae8bdb8095d390b4ff69a2becc6e"


Installing NiTE2 - A middleware library that calculates skeleton
----------------------------------------------------------------

1. Get NiTE2 library from http://ilab.usc.edu/packages/forall/ubuntu-13.10/NiTE-Linux-x64-2.2.tar.bz2

2. Extract the NiTE-Linux-x64-2.2 file from it

3. Run `install.sh` script provided inside NiTE-Linux-x64-2.2
   ::

        cd NiTE-Linux-x64-2.2
        sudo ./install.sh

4. Copy **OpenNI2-FreenectDriver/build/libFreenectDriver.so** to
   **NiTE-Linux-x64-2.2/Samples/Bin/OpenNI2/Drivers/**

5. If everything went okay then you should be able to run the test app
   UserViewer under NiTE-Linux-x64-2.2/Samples/Bin/ to see whether you can get
   your skeleton extracted.

Running demo tracker from body tracking application
---------------------------------------------------

1. Get body_tracker from
   https://bitbucket.org/italianmarssociety/eras/src/4b81655f574ac134bc0fa34ebc5032aa00369cbd/servers/body_tracker/tracker/?at=default

2. Make a copy of NiTE2 directory(**path-to/NiTE-Linux-x64-2.2/Samples/Bin/NiTE2**)
   inside **body_tracker/tracker** directory

3. Before running **make** command make sure your body_tracker directory
   contains OpenNI2 and NiTE2 directories which can be downloaded from here:

   | OpenNI2 -
     https://drive.google.com/file/d/0B7G0kTW5l8bxMUtLQlRKSndMS1E/edit?usp=sharing
   | NiTE2   -
     https://drive.google.com/file/d/0B7G0kTW5l8bxS253eXdCanc5Qkk/edit?usp=sharing

   .. note::

      * They contains only drivers and libraries. They are slightly different
        from what you installed before.
      * Once you get SkeletonTracker.so file presence of these two directories
        are not required any more.


4. Run **make** command inside tracker directory and copy generated `*.so` file
   to path-to/NiTE-Linux-x64-2.2/Samples/Bin/

5. Run
   ::

        export LD_LIBRARY_PATH=path-to/NiTE-Linux-x64-2.2/Samples/Bin

   .. note::

      Better copy above command in .bashrc file as it prevents you from running
      this command everytime you spawn a new shell to run your application

6. Now, run test_userSD.py file as
   ::

        python test_userSD.py

7. Application will start. Trying moving a bit to get tracked.


Troubleshooting
...............

* **The sensor is not found**
    * Run **freenect-glview** command in terminal to check status of kinect
      if you get a RGB-Depth images then your kinect is working. Else if you
      get output as::

        Kinect camera test
        Number of devices found: 0

      This means your Kinect device cann't be detected.

    - **Check that kinect is properly plugged to your PC/laptop and adaptor
      power supply is on**

    * Try unplugging the sensor and plugging it back again, then wait for
      few seconds (10 to be sure).
    * Check last lines of `dmesg` output to see if the device shows up
    * Try running the program with sudo
    * If it works then, make sure you've either logged out and back in
       again after you added your user to the `video` group, or type
       `newgrp video` and see if you can then run it as normal user.

*  **OSError: *.so: cannot open shared object file: No such file or
   directory**

   - Make sure you have exported LD_LIBRARY_PATH before running application and
     it points to right location


* **Could not find data file ./NiTE2/s.dat**

  if you get following error

  ::

      Could not find data file ./NiTE2/s.dat
      current working directory = /home/abhishek/Traking
      write_register: 0x0006 <= 0x00
      OpenNI2-FreenectDriver: Closing device freenect://0
      Couldn't create user tracker


  Make sure NiTE2 directory lies in the current directory(**look at Step 2**)
  from where application is run.
