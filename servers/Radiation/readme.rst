Readme
======

This draft document provides instructions for installation of the
dependencies, and also running the code.

Server Side
===========

Setup MongoDB
-----------

Follow the 4 installation steps for `MongoDB. <https://docs.mongodb.com/tutorials/install-mongodb-on-ubuntu/>`_


Installing the Dependencies
---------------------------

For installing the dependencies::

	pip install -r requirements.txt


Running the Radiation Forecasting Server
----------------

Running the startercode.py for hosting the server::

	python startercode.py
	

Example code
------------

Here is the screen once hosting is successful `plot. <image.png/>`_

 	
Contents of the client_output
-----------------------------

time of arrival::

	None            :-  when the SEP probability threshold(provided by forspef) is below threshold

	Time in seconds :-  when the probability is above the threshold value

prediccs-alarm::
	
	None       :- When the radiation dosage is below the threshold 

	Warning!!! :- When the radiation dosage is below the threshold 

	
all-clear	::
	
	None       :- When there is no  event occuring

	all-clear  :- When the event has passed 

	
	
