Readme
======

This draft document provides instructions for installation of the
dependencies, and also running the code.


Setup Git
---------

You need to have Git installed on your system for cloning PyBrain::
	
	sudo apt-get install git
	
For systems other than Debian/Ubuntu, you can use the corresponding
package managers. Additional Git installation instructions can be
found at `Git-scm. <http://git-scm.com/downloads>`_


Setup SciPy
-----------

For Ubuntu::

	sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose

For other systems, the installation instructions are available at
`SciPy. <http://www.scipy.org/install.html>`_


Setup PyBrain
-------------

Make sure you have SciPy installed, then Clone the PyBrain library from 
Github using this command::

	git clone git://github.com/pybrain/pybrain.git

From the cloned directory, install PyBrain using::

	python setup.py install


Running the Code
----------------

Data Retriever unit::

	python dataretriever.py /home/Repos/eras/servers/solarstorm/database/data

Provide the destination directory as argument. The raw data files will
be downloaded to the provided directory.

Parser unit::

	python parser.py /home/Repos/eras/servers/solarstorm/database/data
	
Provide the source directory, where you downloaded the data files in the
previous step. This unit will parse the data files in the source directory
and generate the CSV file.

Neural Network Training::

	python neuraltraining.py

This should be in same directory as the parser.py It will automatically
take the CSV file as input, train the neural network and generate the
XML file as output.
