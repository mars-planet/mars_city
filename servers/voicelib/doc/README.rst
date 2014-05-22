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


2. Sphinxbase
=============

Download Sphinxbase from
http://sourceforge.net/projects/cmusphinx/files/sphinxbase/0.8/
 
Installation
~~~~~~~~~~~~

::

   cd sphinxbase-0.8
   configure; make; sudo make install
   cd python
   python setup.py build
   python setup.py install
   
   OR
   
   sudo apt-get install sphinxbase-utils

NOTE:sphinxbase has to be installed first before installing pocket sphinx

3. Pocketsphinx
===============

download pocketsphinx 0.8 from
http://sourceforge.net/projects/cmusphinx/files/pocketsphinx/0.8/

Installation
~~~~~~~~~~~~

::
   Manual installation
   cd pocketsphinx-0.8
   configure; make; sudo make install
   cd python
   python setup.py build
   python setup.py install

   OR
   sudo apt-get install python-pocketsphinx
   sudo apt-get install pocketsphinx-hmm-wsj1
   sudo apt-get install pocketsphinx-lm-wsj

Speechrecognition(pep8).py
==========================

Input==>  voice recording of 3 seconds
Output==> Recorded wav file of 3 seconds called livewav.wav
          Speech is recognised from the wav file and text is displayed on the terminal.
 




