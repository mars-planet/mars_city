Instructions:

The following instructions were tested on Win10 X64 version. All the software used was x86 version.

1: Installation of MySQL: http://dev.mysql.com/downloads/mysql/

While installation we cannot select a manual destination folder and Mysql is installed in C:\Program Files (x86)\MySQL by default.

During installation it is mandatory to set at least a 4 character password for root which wasn't the case for previous versions.

This is against the recommendation from tango-controls which was specific for older version of SQL.

2: Installation of TANGO

You can download a ready to use binary distribution on this site: http://www.tango-controls.org/downloads/binary/

Execute the installer. You should specify the destination folder created before: 'C:\tango'

After installation you can edit the MySQL password.

3: Configuration

3-1 TANGO-HOST

Define a TANGO_HOST environment variable containing both the name of the host on which you plan to run the TANGO database device server (e.g. myhost) and its associated port (e.g. 20000). With the given example, the TANGO_HOST should contain myhost:20000.

On Windows you can do it simply by editing the properties of your system. Select 'Advanced -&gt; Environment variables'.

3-2 Create environment variables.

2 new environment variables has to be created to run create-db.bat

3-2-1 MYSQL_USER : this should be root

3-2-2 MYSQL_PASSWORD : fill in the password which was used during mysql installation.

3-3 MODIFY PATH Environment Variable

Add this to windows path for running sql queries.

C:\Program Files (x86)\MySQL\MySQL Server 5.7\bin

3-4 Create the TANGO database tables

Be sure the mysql server is running, normally it should.

Execute %TANGO_ROOT%\share\tango\db\create_db.bat.

3-5 Start the TANGO database:

execute %TANGO_ROOT%\bin\start-db.bat test -v4

the console should show

"Ready to accept request" on successful installation.

3-6 Start JIVE

Now you can test TANGO with the JIVE tool, from the Tango menu, or by typing the following command on a DOS windows :

%TANGO_ROOT%\bin\start-jive.bat

Ref: http://www.tango-controls.org/resources/howto/how-install-tango-windows/