from __future__ import print_function

from pyEUROPA.engine import stopPSEngine, makePSEngine
from pyEUROPA import Actor
from plan import Plan

import os
import logging
from PyTango import DeviceProxy

class Planner(object):

    def __init__(self, device):

        # Current Planning Device
        self.device = device

        # Connect to EUROPA
        self.europa = makePSEngine("g")
        self.europa.start()

        # Initial State of Rover
        self.europa.executeScript("nddl",
            'initial-state.nddl', True);

        # Execute the rover's goals (right now, to go to location)
        self.europa.executeScript("nddl",
            'GOALS.nddl', True);

        # Create planning logger
        log_file = 'plans.log'
        open(log_file,"w").close() # clear old log
        self.logger = logging.getLogger('Planner')
        hdlr = logging.FileHandler(log_file)
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.INFO)

        ### TEST ###
        cp = self.getCurrentPlan() # Get the current plan
        self.logger.info(cp.log)
        

        self.executePlan(cp,"myro")

    def executePlan(self, plan, target=None):
        """Sends plan to the specified target for plan execution"""

        if target is None:
            raise ValueError("Must specify target where\
                plan should be executed!")

        elif target=="gazebo":
            # TODO: Execute plan on Gazebo Controls
            pass

        elif target=="myro":
            # Execute plan on myro
            myro = DeviceProxy("c3/rovers/myro")
            

            # TODO: Find way to move on Myro

        else:
            raise ValueError("%s execution target not-defined!"%target)


    def getCurrentPlan(self):
        
        # Generate plan in current enviroment
        self.generatePlans()

        # The EUROPA Plan Database
        europa_log = self.europa.planDatabaseToString()
        plan = Plan(europa_log) 

        return plan

    def generatePlans(self):
        """Create plan through EUROPA based on current environment"""

        solver = self.europa.createSolver("PlannerConfig.xml");
        solver.configure(0,1000);
        solver.solve(200,200);
