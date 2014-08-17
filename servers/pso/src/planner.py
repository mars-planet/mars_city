from __future__ import print_function

# Temporary snippet for debugging
import sys
sys.path.remove("/usr/local/lib/python2.7/dist-packages/pyEUROPA-0.1-py2.7.egg")
sys.path.append("/home/mathew/Desktop/pyEUROPA")

from pyEUROPA.engine import stopPSEngine, makePSEngine
from pyEUROPA import Actor
from plan import Plan

import os
import json
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

        # Set the rover's goals (right now, to go to sample nearby rocks)
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
        self.objects = cp.objects
        self.actions = cp.actions
        self.logger.info("Objects in Enviroment "+str(cp.objects))
        self.logger.info("Plan Actions "+str(cp.actions))
        

    def get_objects(self):
        """Objects considered in current plan"""

        return json.dumps(self.objects)

    def get_actions(self):
        """Actions considered in current plan"""

        return json.dumps(self.actions)

    def getCurrentPlan(self):
        
        # Generate plan in current enviroment
        self.generatePlans()

        # Extract the EUROPA Plan directly from the PLASMA Database
        europa_log = self.europa.planDatabaseToString()
        plan = Plan(europa_log) 

        return plan

    def generatePlans(self):
        """Create plan through EUROPA based on current environment"""

        solver = self.europa.createSolver("PlannerConfig.xml");
        solver.configure(0,1000);
        solver.solve(200,200);
