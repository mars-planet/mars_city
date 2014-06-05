===========================================
Software Architecture Document for the Planning and Scheduling on the Trevor Rover  
===========================================

:Author: Mathew Kallada


Change Record
=============

2014.06.04 - Document created.

Architectural Requirements 
==========================
In order for pyEUROPA to run correctly you must have the following software:

- The python module py4j
- EUROPA-pso


Overview
--------

The pyEUROPA python module exposes the PSEngine [EUROPA's primary interface] 
to Python. This has been done through py4j. 