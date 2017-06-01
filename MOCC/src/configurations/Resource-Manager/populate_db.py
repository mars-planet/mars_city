"""
Use this to populate the sb for testing
"""
from models import Resources
import datetime


def init_db():
    arbit_time = datetime.datetime.now()

    resources = [{"name": "oxygen",
                 "type": "consumable",
                  "availability_start": str(arbit_time),
                  "availability_end": str(arbit_time),
                  "amount": "123.123",
                  "rate": "12.34"
                  },
                 {"name": "water",
                  "type": "consumable",
                  "availability_start": str(arbit_time),
                  "availability_end": str(arbit_time),
                  "amount": "5254.2342",
                  "rate": "74.443"
                  },
                 {"name": "battery",
                  "type": "energy",
                  "availability_start": str(arbit_time),
                  "availability_end": str(arbit_time),
                  "amount": "423",
                  "rate": "1135.3"
                  }
                 ]
    Resources.insert_many(resources).execute()


if __name__ == '__main__':
    init_db()
