==========================================================
Docker Container Instructions for Telerobotics
==========================================================

:Author: Siddhant Shrivastava

.. contents::
   :local:
   :depth: 2

Change Record
=============

.. If the changelog is saved on an external file (e.g. in servers/sname/NEWS),
   it can be included here by using (dedent to make it work):

25\ :sup:`th`\  May, 2015 - Document created.

  .. literalinclude:: ../../servers/servername/NEWS


Introduction
============

From the `Docker website <https://www.docker.com/>`_ -

    Docker is an open platform for building, shipping and running distributed applications. It gives programmers, development teams and operations engineers the common toolbox they need to take advantage of the distributed and networked nature of modern applications.

The Telerobotics Docker Image is a Ubuntu 14.04 image configured with Robot Operating System (ROS) packages, Tango-Controls, and Gazebo models to use Telerobotics out-of-the-box. ERAS is also provided as part of the image.

Docker Installation
=============

Install Docker for your Platform by following instructions from `this site <https://docs.docker.com/installation/>`_

Setting up the Telerobotics Image
=====================

Pull the ``sidcode/ros-eras`` image the Docker Hub by running the following command -

``docker pull sidcode/ros-eras``

Note that a high-speed Internet connection is required as the image is around 4 GB in size.

Using the Telerobotics Image
========================

Most of the features of the image can be harnessed using the following command -

``docker run -i -t sidcode/ros-eras bash``

In order to run ``Gtk-based`` applications, one needs to share the display with the host machine. To achieve this, start the image with the following command -

``docker run -i -t -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix sidcode/ros-eras /bin/bash``

This should provide access as a root user to the Telerobotics image.

Start the ssh-server which is to be used for ROS GUI applications -

``./etc/init.d/ssh start``

A Linux group called ``eras`` and a user called ``eras`` is configured in the image. The details are as follows -

  Username - ``eras``
  Password - ``eras``

View the Network interface information by running ``ifconfig``. The IPv4 Address corresponding to the ``eth0`` interface is the one that is to be used to connect to the Docker image.

From the Physical machine, start a terminal and ``ssh`` to the Docker image as follows -

``ssh -Y <docker-IP> -l eras``

Now we are all set to use the Telerobotics Docker Image.

Follow the instructions in the rest of the manuals for the next steps.
