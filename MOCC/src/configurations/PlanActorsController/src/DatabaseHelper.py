from __future__ import division, print_function
from plan_actor_schema import PlanActors
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker


# Connecting to the database
engine = create_engine('sqlite:///plan_actors.db', echo=False)
Session = sessionmaker(bind=engine)

# Create session
s = Session()


# To add the plan actor fields into the database
def add(data):
    address = data[0]  # Address
    type_t = data[1]  # Type
    avail_start = data[2]  # available start date time
    avail_end = data[3]  # available end date time
    capabilities = data[4]  # capability string - JSON

    query = s.query(PlanActors).filter(PlanActors.address.in_([address]))
    result = query.first()
    if result:
        return -1
    else:
        actor = PlanActors(address, type_t, avail_start,
                           avail_end, capabilities)
        s.add(actor)

        # commit the record the database
        s.commit()

    print("Inserted row successfully")
    return 0


# To retrive the details of the actor whose address is passed
def get(address):
    return_data = []

    query = s.query(PlanActors).filter(PlanActors.address.in_([address]))
    result = query.first()

    return_data.append(result.address)
    return_data.append(result.type)
    return_data.append(result.avail_start)
    return_data.append(result.avail_end)
    return_data.append(result.capabilities)
    return return_data
