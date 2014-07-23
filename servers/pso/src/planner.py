from __future__ import print_function
from pyEUROPA.psengine import stopPSEngine, makePSEngine
from plan import Plan

import os
import logging

class Planner(object):

    def __init__(self, device):

        self.device = device
        cwd = os.getcwd()

        # Connect to EUROPA
        print('Connecting to EUROPA platform via pyEUROPA')
        self.europa = makePSEngine("g")
        print('Launching PSEngine Instance')
        self.europa.start()

        errors = self.europa.executeScript("nddl",
            'initial-state.nddl', True);

        if (errors!=""):
            raise ValueError("Failed loading model:"+errors);

        # Create planning logger
        self.logger = logging.getLogger('Planner')
        hdlr = logging.FileHandler('plans.log')
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.INFO)

        ### TEST ###
        cp = self.getCurrentPlan() # Get the current plan
        self.logger.info(cp)

    def executePlan(self, plan, target=None):
        """Sends plan to the specified target for plan execution"""

        if target is None:
            raise ValueError("Must specify target where\
                plan should be executed!")
        elif target=="gazebo":
            # TODO: Execute plan on Gazebo Controls
            pass
        else:
            raise ValueError("%s execution target not-defined!"%target)

    def modifyEnviroment(self, type="add"):
        """Modify the current enviroment that the rover is located in."""
        pass


    def getCurrentPlan(self):
        
        # Generate plan in current enviroment
        self.generatePlans()

        europa_log = self.europa.planDatabaseToString()
        plan = Plan(europa_log) 

        return plan

    def generatePlans(self):
        """Create plan through EUROPA based on current environment"""

        solver = self.europa.createSolver("PlannerConfig.xml");
        solver.configure(0,1000);
        solver.solve(200,200);
