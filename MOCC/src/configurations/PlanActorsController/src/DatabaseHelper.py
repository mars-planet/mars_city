import sqlite3 as sq


# Connection to the database
con = sq.connect('plan_actors', check_same_thread=False)

# To create the initial database, to be called only once from the service
def createDatabase():
    con.execute('DROP TABLE IF EXISTS plan_actors;')
    con.execute("CREATE TABLE plan_actors\
    (address TEXT PRIMARY KEY NOT NULL,\
    type TEXT NOT NULL,\
    avail_start DATE NOT NULL,\
    avail_end DATE NOT NULL,\
    capabilities TEXT NOT NULL);")
    print("Table created successfully")


# To add the plan actor fields into the database
def add(data):
    address = data[0]   #Address
    type_t = data[1]    #Type
    avail_start = data[2]   #available start date time
    avail_end = data[3]     #available end date time
    capabilities = data[4]  #capability string - JSON

    query = '''INSERT INTO plan_actors VALUES
    (\'''' + address + '''\',
    \'''' + type_t + '''\',
    \'''' + avail_start + '''\',
    \'''' + avail_end + '''\',
    \'''' + capabilities + '''\');'''
    try:
        con.execute(query)
        con.commit()
    except sq.IntegrityError:
        return -1

    print("Inserted row successfully")
    return 0


#To retrive the details of the actor whose address is passed
def get(address):
    return_data = []

    query = '''SELECT * FROM plan_actors WHERE
    ADDRESS == \'''' + address + '''\';'''
    cursor = con.execute(query)

    for row in cursor:
        return_data.append(row[0])
        return_data.append(row[1])
        return_data.append(row[2])
        return_data.append(row[3])
        return_data.append(row[4])
    return return_data
