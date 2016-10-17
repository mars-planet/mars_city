#!/usr/bin/env python

'''
setup.py file for SWIG eHealth module
'''

from setuptools import setup, find_packages


setup(name='Health Monitor GUI',
      version='0.1',
      author='Mario Tambos',
      package_data={'': ['*.cfg', '*.png']},
      packages=find_packages(exclude='test'),
      zip_safe=False,
      install_requires=['matplotlib>=1.3.1',
                        'numpy>=1.8.1',
                        'pandas>=0.14.0',
                        'PyTango>=8.1.2',
                        'setuptools>=3.6',
                        'wxmplot>=0.9.14',
                        'wxpython>=3.0']
      )
