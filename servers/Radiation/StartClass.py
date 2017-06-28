import os
from pymongo import MongoClient
from datetime import datetime


class start():

    '''
        Includes the main logic for the deterministic approach
        Definitions of Class variables:
            Al_3_threshold		 		:	Thin spacesuit shielding threshold
                                            (Al 0.3 g/cm2)[H2O 10g/cm2]
            Al_10_threshold		 		:	Storm shelter shielding	threshold
                                            (Al 10.0 g/cm2)[H2O 10g/cm2]
            SEP_pt                  	:	The SEP probability threshold
                                            for FORSPEF
            Distance					:	The Earth Sun distance
            clear_limit					:	Number of dosage readings below
                                            threshold for the all-clear signal
    '''
    Al_3_threshold = 0.068
    Al_10_threshold = 0.068
    SEP_pt = 0.25
    Distance = 149600000
    stack = []
    stack2 = []
    alarm_triggered = 0
    SEP = 0
    no_of_times = 0
    clear_limit = 3
    last_data = 0.0
    counter = 0
    flag = 0

    def __init__(self, db):
        client = MongoClient()
        self.db = client[db]

    def all_clear(self):
        '''
            Determines the end of the SEP event by analysing the PREDICCS data.
        '''
        print "\nall-clear triggered"
        count = 0
        for i in self.db['prediccs'].find().sort('_id', -1).limit(1):
            last_entry = i
        for i in last_entry['data']:
            if count:
                self.stack.append(i)
                if float(i[-2]) < self.Al_3_threshold:
                    self.stack2.append(i[0])
                    if len(self.stack2) == self.clear_limit:
                        print "all-clear"
                        self.SEP = 0
                        print "stack", self.stack2
                        self.stack = []
                        self.stack2 = []
                        self.last_data = i[0]
                        return 1
                        break
                else:
                    self.stack2 = []

            if i[0] == self.stack[-1][0]:
                count = 1

    def prediccs_alarm(self):
        '''
            The method analyses the PREDICCS data and provides a
            forecast in a lower window.
        '''
        print "\nprediccs-alarm triggered"
        self.flag = 0
        stack = []
        count = 0
        for i in self.db['prediccs'].find().sort('_id', -1).limit(1):
            last_entry = i
        # incase if there is another SEP event and to avoid the already
        # detected one
        for i in last_entry['data']:
            if self.last_data == i[0]:
                count = 1
            if count:
                if float(i[-2]) > self.Al_3_threshold or self.lag == 1:
                    stack.append(i)
                    self.flag = 1

        if not count:
            for i in last_entry['data']:
                if float(i[-2]) > self.Al_3_threshold or self.flag == 1:
                    stack.append(i)
                    self.flag = 1

        if self.flag:
            self.stack = stack
            self.SEP = 1
            if self.no_of_times == 1:
                self.alarm_triggered = 0
            self.no_of_times = self.no_of_times - 1
            return 1
        else:
            return 0

    def alarm(self):
        '''
                The method checks FORSPEF data and checks
                if the SEP probability is above the threshold
                and transfers control to prediccs_alarm method.
                The method provides an higher window of prediction.
        '''
        print "\nalarm triggered"
        for i in self.db['forspef'].find().sort('_id', -1).limit(1):
            last_entry = i
        probs = []
        for i in last_entry['data']:
            probs.append(i[2])
        if float(max(probs)) > self.SEP_pt:
            self.alarm_triggered = 1
            self.no_of_times = self.no_of_times + 1
            for i in self.db['cactus'].find().sort('_id', -1).limit(1):
                last_entry = i
            max_velocity = []
            for i in i['data']:
                max_velocity.append(i[-1])
            time = self.Distance / int(max(max_velocity))
            return time

    def plots(self):
        '''
            The method provides the data for the plot
        '''
        data = []
        for i in self.db['prediccs'].find():
            if not data:
                for j in i['data']:
                    data.append(j)
            else:
                flag = 0
                final_entry = data[-1][-1][0]
                for j in i['data']:
                    if flag:
                        data.append(j)
                    if final_entry == j[0]:
                        flag = 1
        return {'data': data}
