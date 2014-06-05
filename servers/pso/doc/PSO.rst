===========================================
Software Architecture Document for the Planning and Scheduling on the Trevor Rover  
===========================================

:Author: Mathew Kallada


Change Record
=============

2014.06.04 - Document created.

Goals
==========================

- To scheudle and manage plans through EUROPA.


Architectural Requirements 
==========================

In order for pyEUROPA to run correctly you must have the following software:

- The python module py4j
- EUROPA-pso


Overview
--------


pyEUROPA
--------

The pyEUROPA python module exposes the PSEngine [EUROPA's primary interface] 
to Python. This has been done through py4j. 


   from pyEUROPA.psengine import makePSEngine, stopPSEngine

    # Launch & connect to EUROPA
    europa = makePSEngine()
    europa.start()
    europa.executeScript("nddl","some_file.nddl",true)

    # Shuts down all PSEngine instances
    stopPSEngine()