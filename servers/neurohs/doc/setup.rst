==========================
Neuro Headset Server Setup
==========================

.. highlightlang:: sh

Dependencies
============

The following dependencies are required to use the Neuro Headset::

   sudo apt-get install python-setuptools python-dev realpath
   wget https://github.com/openyou/emokit/archive/master.zip
   unzip master.zip
   cd emokit-master/python/
   sudo python setup.py install


Running the server
==================

To run the stand-alone server use::

  $ python neurohs.py 400

the argument (e.g. 400) is the polling period in milliseconds.

----

To run the server from Tango use::

  $ python neurohs epoc1

The argument (e.g. epoc1) is the server name registered in Jive.


Troubleshooting
===============

If you installed all the dependencies, everything should work.

If you see this error::

  emokit-master/python$ sudo python setup.py install
  Traceback (most recent call last):
    File "setup.py", line 2, in <module>
      from setuptools import setup
  ImportError: No module named setuptools

you have to ``sudo apt-get install python-setuptools``.

----

If you see this error::

  emokit-master/python$ sudo python setup.py install
  ...
  config.status: executing src commands
  source/_ctypes.c:107:20: fatal error: Python.h: No such file or directory
  compilation terminated.
  error: Setup script exited with error: command 'i686-linux-gnu-gcc' failed with exit status 1

you have to ``sudo apt-get install python-dev``.

----

If you see this error::

  >>> headset.setup()
  Traceback (most recent call last):
    File "<stdin>", line 1, in <module>
    File "/usr/local/lib/python2.7/dist-packages/emokit-0.0.1-py2.7.egg/emokit/emotiv.py", line 353, in setup
      self.setupPosix()
    File "/usr/local/lib/python2.7/dist-packages/emokit-0.0.1-py2.7.egg/emokit/emotiv.py", line 450, in setupPosix
      setup = self.getLinuxSetup()
    File "/usr/local/lib/python2.7/dist-packages/emokit-0.0.1-py2.7.egg/emokit/emotiv.py", line 370, in getLinuxSetup
      realInputPath = check_output(["realpath", "/sys/class/hidraw/" + filename])
    File "/usr/lib/python2.7/subprocess.py", line 568, in check_output
      process = Popen(stdout=PIPE, *popenargs, **kwargs)
    File "/usr/lib/python2.7/subprocess.py", line 711, in __init__
      errread, errwrite)
    File "/usr/lib/python2.7/subprocess.py", line 1308, in _execute_child
      raise child_exception
  OSError: [Errno 2] No such file or directory

you have to ``sudo apt-get install realpath``.

----

If you see this error::

  $ python neurohs.py 400
  Serial: SNxxx Device: hidraw1
  Serial: SNxxx Device: hidraw2 (Active)
  Traceback (most recent call last):
    File "/usr/lib/python2.7/dist-packages/gevent/greenlet.py", line 390, in run
      result = self._run(*self.args, **self.kwargs)
    File "/usr/local/lib/python2.7/dist-packages/emokit-0.0.1-py2.7.egg/emokit/emotiv.py", line 353, in setup
      self.setupPosix()
    File "/usr/local/lib/python2.7/dist-packages/emokit-0.0.1-py2.7.egg/emokit/emotiv.py", line 453, in setupPosix
      self.hidraw = open("/dev/" + setup[1])
  IOError: [Errno 13] Permission denied: '/dev/hidraw2'

you need to run ``sudo python neurohs.py 400``.
