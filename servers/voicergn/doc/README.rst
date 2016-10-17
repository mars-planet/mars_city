============================
Voicelib - Setup and Running
============================

1. Prerequisites
================

install 
	numpy,
	scipy,
	ipython(optional)
	pyaudio
	
	using apt-get install python-"package name" or pip install package name

Run importacoustic.sh in order to store adapted hmm model in home directory. 


2. Sphinxbase 
=============

::

  wget http://downloads.sourceforge.net/project/cmusphinx/sphinxbase/0.8/sphinxbase-0.8.tar.gz
  wget http://downloads.sourceforge.net/project/cmusphinx/pocketsphinx/0.8/pocketsphinx-0.8.tar.gz
  tar -zxvf sphinxbase-0.8.tar.gz
  tar -zxvf pocketsphinx-0.8.tar.gz
 
Installation
~~~~~~~~~~~~

::

   cd ~/sphinxbase-0.8/
   ./configure --enable-fixed
   make
   sudo make install
   
   OR
   
   sudo apt-get install sphinxbase-utils

NOTE:sphinxbase has to be installed first before installing pocket sphinx

3. Pocketsphinx
===============


Installation
~~~~~~~~~~~~

::

   cd ~/pocketsphinx-0.8/
   ./configure
   make
   sudo make install

   OR
   sudo apt-get install python-pocketsphinx
   sudo apt-get install pocketsphinx-hmm-wsj1
   sudo apt-get install pocketsphinx-lm-wsj
   

4. Speechrecognition(pep8).py
=============================
Input
~~~~~

Voice recording of 3 seconds.

Output
~~~~~~

Recorded wav file of 3 seconds called livewav.wav
Speech is recognised from the wav file and text is displayed on the terminal.
 

5. Raspberry pi setup.
======================

Expand filesystem
~~~~~~~~~~~~~~~~~

sudo raspi-config


Installing required tools.
~~~~~~~~~~~~~~~~~~~~~~~~~~
::

  sudo apt-get update
  sudo apt-get upgrade --yes
  sudo apt-get install vim git-core espeak python-dev python-pip bison libasound2-dev libportaudio-dev python-pyaudio --yes


Plug in your USB microphone. Let’s open up an ALSA configuration file in nano:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

sudo vim /etc/modprobe.d/alsa-base.conf

Comment the line by adding #.

#options snd-usb-audio index=-2

press ^o or ctrl+o to save.
press ^x or ctrl+x to exit.

Reboot alsa.
~~~~~~~~~~~~
::

   sudo alsa force-reload

Install other important packages.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  sudo pip install -r ../eras/servers/voicelib/requirements.txt
  cd ./eras/servers/voicelib/voice_recognition
  sudo bash importacoustic.sh
