#!/usr/bin/env python

"""
setup.py file for SWIG eHealth module
"""

from distutils.core import setup, Extension


eHealth_module = Extension('_eHealth',
                           sources=['eHealth.i', 'arduPi.cpp', 'eHealth.cpp'],
                           swig_opts=['-c++'],
                           include_dirs=['./'],
                           libraries=['pthread', 'rt']
                           )

setup(name='eHealth interface module',
      version='0.1',
      author="Coocking Hacks & Mario Tambos",
      description="""Module to read data from the e-Health shield for Arduino
                     and Raspberry Pi""",
      ext_modules=[eHealth_module],
      )
