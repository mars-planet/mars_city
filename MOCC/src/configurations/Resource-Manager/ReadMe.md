Execute the scripts in the following order:

1. resource_manager_ds_setup.py : to set up the test resource manager device server

2. python ResourceManagerDS.py test  : to run the Resource Manager device server

3. TestResourceManager.py [in another console] : to unit test the resource manager


Note: SQLite database is already provided. 
		In case of errors run models.py and then populate_db.py

Current Methodology:

A Resources database is made in SQLite

A device server called ResourceManagerDS provides access to this database through the command ask_resource(resource_name)

currently, data about the resource is being transferred as a string as we cannot pass python objects through PyTango commands.
This could be improved.

Fields of the data are seperated by ' # '

Relevant information will be extracted from the string.

Further changes to be made after review.