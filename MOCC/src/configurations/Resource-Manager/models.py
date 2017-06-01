"""
Defines the Resources table and
creates it if the script is called
"""

import peewee

db = peewee.SqliteDatabase("Resources.db")


class BaseModel(peewee.Model):
    class Meta:
        database = db


class Resources(BaseModel):
    """
    ORM model for Resources table
    """
    name = peewee.CharField(primary_key=True)
    type = peewee.CharField(null=False)
    availability_start = peewee.DateTimeField()
    availability_end = peewee.DateTimeField()
    amount = peewee.DecimalField()
    rate = peewee.DecimalField()


if __name__ == "__main__":
    try:
        Resources.create_table()
    except peewee.OperationalError:
        print "Resources table already exists!"
