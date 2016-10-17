========================================================================
Tango Installation on Windows 10
========================================================================


:Author: Shridhar Mishra

Change Record
=============

- 10 July 2016 Created.
- 21 August 2016

Introduction
============
Tango installation on windows can be a bit confusing since the documentation on the website is old and there are a
few changes with the new versions of MYSQL.

Here's the installation procedure that can be helpful.

Installation steps
==================

1. Installation of MySQL
------------------------

In this installation process mysql 5.7.11 is installed. Newer versions can also be installed.
while installation we cannot select a manual destination folder and Mysql is installed in C:\Program Files (x86)\MySQL by default.
During installation it is mandatory to set at least a 4 character password for root which wasn't the case for previous versions.
This is against the recommendation from tango-controls which was specific for older version of SQL.

2. Installation of TANGO
------------------------
You can download a ready to use binary distribution on this site.

http://www.tango-controls.org/downloads/binary/

Execute the installer. You should specify the destination folder created before :'c:\tango'
After installation you can edit the MySQL password.

3. Configuration
-----------------

    - **3.1 TANGO-HOST**

        Define a TANGO_HOST environment variable containing both the name of the host on which you plan to run the
        TANGO database device server (e.g. myhost) and its associated port (e.g. 20000). With the given example,
        the TANGO_HOST should contain myhost:20000.

        On Windows you can do it simply by editing the properties of your system.
        Select 'Advanced', 'Environment variables'.

    - **3.2 Create environment variables.**

     2 new environment variables has to be created to run create-db.bat
        **3-2-1 MYSQL_USER**

        This should be ```root```

        **3-2-2 MYSQL_PASSWORD**

        Fill in the password which was used during mysql installation.


    - **3.3 MODIFY PATH**

    Add this to windows path for running sql queries.
    C:\Program Files (x86)\MySQL\MySQL Server 5.7\bin

    - **3.4 Create the TANGO database tables**

    Be sure the mysql server is running, normally it should.
    Execute %TANGO_ROOT%\share\tango\db\create_db.bat.

    - **3.5 Start the TANGO database:**

    execute %TANGO_ROOT%\bin\start-db.bat -v4
    the console should show
    "Ready to accept request" on successful installation.

    - **3.6 Start JIVE**

    Now you can test TANGO with the JIVE tool, from the Tango menu, or by typing the following command on a DOS windows :
        %TANGO_ROOT%\bin\start-jive.bat


Ref: `http: www.tango-controls.org/resources/howto/how-install-tango-windows/ <http: www.tango-controls.org/resources/howto/how-install-tango-windows/>`_