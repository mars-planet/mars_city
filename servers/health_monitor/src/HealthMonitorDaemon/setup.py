#!/usr/bin/env python

"""
setup.py file for SWIG eHealth module
"""

from setuptools import setup, find_packages


setup(name='Health Monitor Daemon',
      version='0.1',
      author='Mario Tambos',
      package_data={'': ['*.cfg']},
      packages=find_packages(exclude='test'),
      zip_safe=False,
      install_requires=['inflect>=0.2.4',
                        'numpy>=1.8.1',
                        'MDP>=3.3',
                        'Oger>=1.1.3',
                        'pandas>=0.14.0',
                        'PyTango>=8.1.2',
                        'scipy>=0.14.0']
      )
