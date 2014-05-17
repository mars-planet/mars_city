
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

download Sphinxbase from
http://sourceforge.net/projects/cmusphinx/files/sphinxbase/0.8/
 
installation
~~~~~~~~~~~~

::

   cd sphinxbase-0.8
   configure; make; sudo make install
   cd python
   python setup.py build
   python setup.py install

NOTE:sphinxbase has to be installed first before installing pocket sphinx

3. pocketsphinx
===============

download pocketsphinx 0.8 from
http://sourceforge.net/projects/cmusphinx/files/pocketsphinx/0.8/

installation
~~~~~~~~~~~~

::

   cd pocketsphinx-0.8
   configure; make; sudo make install
   cd python
   python setup.py build
   python setup.py install







