Execute the scripts in the following order:

1. db_setup.py : to create the Resources table in the tango database assuming username:root and password is empty

2. resource_manager_ds_setup.py : to set up the test resource manager device server

3. python ResourceManagerDS.py test  : to run the Resource Manager device server

4. resource_manager_tester.py [in another console] : to test the resource manager working
													 This could alternatively be done through the console by entering the python console



Current Methodology:

A Resources database is made in MySQL

A device server called ResourceManagerDS provides access to this database through the command ask_resource(resource_name)

currently, data about the resource is being transferred as a string as we cannot pas python objects through commands.
This could be improved.

Relevant information will be extracted from the string.

Further changes to be made after review.