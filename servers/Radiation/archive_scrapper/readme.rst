Readme
======

Scraps data from the archives of `PREDICCS, <http://prediccs.sr.unh.edu/data/goesPlots/archive/>`_ 
`CACTUS <https://secchi.nrl.navy.mil/cactus/>`_  and 
`NOAA <http://www.swpc.noaa.gov/content/data-access/>`_

Running the spiders
===================

Before running the spiders ensure mongod daemon is started

Starting mongod  daemon::
	
	sudo service mongod start

**PREDICCS::**
	
	scrapy crawl prediccs

**CACTUS::**
	
	scrapy crawl cactus

**NOAA::**
	
	scrapy crawl NOAA

Storage location
~~~~~~~~~~~~~~~~

default database is ascrapper (can be changed from the settings), and collections are prediccs,cactus and noaa repectively.