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

Client Side
===========

Install requests package
------------------------

Installing using pip::

	pip install requests


Running the Code
----------------

Open two terminals one for the gunicorn WSGI server and the other one for the 
client. The client script will send a get request to the server in intervals of 
1 hour.

Running the Server::

	gunicorn api:app

Running the client::
	
	python client

For the plot open a new terminal and run::

        python plot
	
Running testapi
---------------

Similar to the api case open two clients one for the client and the
other one for the client 
Running the testServer::

         gunicorn testapi::app
	
Running the client::

         python client


For the plot open a new terminal and run::

        python plot

Example code
------------

Here is an example plot for the test data `plot. <plots_test.pdf/>`_

The output for client `client_output. <client_output.txt/>`_
 	
Contents of the client_output
-----------------------------

thresholds::

	SEP probability threshold

	Thin spacesuit shielding threshold

	Storm shelter shielding threshold

data::
	
	1st entry:- Returns Time of arrival if the FORSPEF probability if above the SEP probability threshold

	2nd entry:- Return 1 if prediccs radiation dosage is above the given threshold else zero or None

	3rd entry:- Return 1 in case of all-clear signal else zero or None
	
time::
	
	Time at which the spiders were called



	
	
