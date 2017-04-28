import sqlite3 as sq

con = sq.connect('plan_actors')

## Create the initial database
## 
## Deletes old and creates new
def createDatabase():
	con.execute('DROP TABLE IF EXISTS plan_actors;')
	con.execute('''CREATE TABLE plan_actors
		(type TEXT NOT NULL,
		address TEXT PRIMARY KEY NOT NULL,
		avail_start DATE NOT NULL,
		avail_end DATE NOT NULL,
		capabilities TEXT NOT NULL);''')
	print("Table created successfully");
	
## Add new entry of the plan_actors into the database
##
## data - dictionary containing necessary field in order
def add(data):
	type_t = data[0]
	address = data[1]
	avail_start = data[2]
	avail_end = data[3]
	capabilities = data[4]
	
	query = '''INSERT INTO plan_actors VALUES
		(\'''' + type_t +'''\',
		\'''' + address +'''\',
		\'''' + avail_start +'''\',
		\'''' + avail_end +'''\',
		\'''' + capabilities +'''\');'''
	con.execute(query)
	
	print("Inserted row successfully")

## Retreives entry from the table where address of plan_actor = address
##
##
def get(address):
	return_data = []
	
	query = '''SELECT * FROM plan_actors WHERE
		ADDRESS == \'''' + address + '''\';'''
	cursor = con.execute(query)	
	
	return cursor




