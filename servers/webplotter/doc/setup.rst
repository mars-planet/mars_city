=================
Web Plotter Setup
=================

.. highlightlang:: sh

The Web Plotter reads data from any Tango server and plots on a graph
on a web page the values of all the attributes provided by the server.


Running the server
==================

To run the web server use::

  $ python webserver.py

This will start running a web server at <http://localhost:8080>.

From the web page you can access any tango device by adding its name
at the end of the URL, e.g. <http://localhost:8080/C3/neurohs/epoc1>.

If the Tango device is up and running, a list of values and a graph
should be displayed.


Usage
=====

The Web Plotter will keep reading data from the Tango server and plot
them in real time.  You can select which attributes are plotted by
checking/unchecking them in the list of attributes.


Troubleshooting
===============

If the list of attributes and the graph are empty, make sure that the
name in the URL is correct and that the server is up and running.
After you started the server you will have to refresh the page.

If the values stop being updated, verify that the server is still
running and then try to refresh the page.
