pyEUROPA: Python bindings for [EUROPA](http://code.google.com/p/europa-pso/)'s PSEngine
========

pyEUROPA is a python wrapper for EUROPA's Java API. The Java API has been embedded in this library.
This wrapper is based on Py4J to communicate with Java library officially provided. 
For native Java API or C++ API, please visit http://code.google.com/p/europa-pso/wiki/EuropaWiki


Install
-------------

1. First, install py4j by running the following command

        pip install py4j

2. Run the following command to install pyEUROPA

        python setup.py install

Example
-------------

        from pyEUROPA.psengine import makePSEngine, stopPSEngine

        # Launch & connect to EUROPA
        europa = makePSEngine("g")
        europa.start()

        # Shuts down all PSEngine instances
        stopPSEngine()


Troubleshooting
-------------

If you get something like `Error: Cannot open shared object file: No such file or directory`

        sudo ldconfig ~/europa/lib
        
or 

        sudo ldconfig YOUR EUROPA DIRECTORY/lib

*Any other problems? Either contact me or make an issue: kallada@cs.dal.ca and I'll help you troubleshoot it.*
