import os
import sys
from setuptools import setup
from pyEUROPA import __version__

setup(
    name="pyEUROPA",
    version=__version__,
    url='https://github.com/kallada/pyEUROPA',
    author='Mathew Kallada',
    author_email='kallada@cs.dal.ca',
    description="A Python wrapper for EUROPA's Java API.",
    long_description='''A Python wrapper for EUROPA-PSO Java API. This wrapper is based on Py4J to communicate with Java library officially provided. For native Java API, please visit http://code.google.com/p/europa-pso/wiki/EuropaWiki''',
    license="LICENSE",
    packages=['pyEUROPA'],
    package_dir={
        'pyEUROPA': 'pyEUROPA'},
    package_data={
        'pyEUROPA': ['java/src/pyEUROPA.java', 'java/lib/py4j0.7.jar', 'java/lib/PY4J_LICENSE',
                     'engine/__init__.py', 'engine/psengine.py', 'engine/py_nddl.py',
                     'interface/__init__.py', 'interface/actor.py', 'interface/enviroment.py',
                     ]
    },
    classifiers=[
        'Development Status :: 1 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: Freely Distributable',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
)
