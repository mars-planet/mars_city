import MySQLdb
import datetime

'''Sets up the initial database required for PyTango'''

# Open database connection
db = MySQLdb.connect("localhost","root","","tango" )

# prepare a cursor object using cursor() method
cursor = db.cursor()

# TODO Not working, fix it.
# Drop table if it already exist using execute() method.
# cursor.execute("DROP TABLE IF EXISTS EMPLOYEE")

# Create table as per requirement
sql_table_create = """CREATE TABLE Resources (
         resource_name  varchar(50) primary key,
         resource_type  varchar(50) not null,
		 availability_start datetime,
		 availability_end  datetime,
		 amount float(10, 6),
		 rate  float(10, 6));"""

cursor.execute(sql_table_create)

# Set up some initial test data

some_datetime = datetime.datetime.now()
cursor.execute("INSERT INTO Resources (resource_name, resource_type, availability_start, availability_end, amount, rate) VALUES (%s, %s, %s, %s, %s, %s)",
   ("Water", "Consumable", some_datetime, some_datetime, 23.4, 32.2))

some_datetime = datetime.datetime.now()
cursor.execute("INSERT INTO Resources (resource_name, resource_type, availability_start, availability_end, amount, rate) VALUES (%s, %s, %s, %s, %s, %s)",
   ("Battery", "EnergyConsumable", some_datetime, some_datetime, 21.2, 52.4))

db.commit()
db.close()