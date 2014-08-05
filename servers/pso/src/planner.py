from __future__ import print_function
from pyEUROPA.psengine import stopPSEngine, makePSEngine
from plan import Plan

import os
import logging
from PyTango import DeviceProxy

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
        log_file = 'plans.log'
        open(log_file,"w").close() # clear old log
        self.logger = logging.getLogger('Planner')
        hdlr = logging.FileHandler(log_file)
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.INFO)

        ### TEST ###
        cp = self.getCurrentPlan() # Get the current plan
        self.logger.info(cp.log)
        

        #self.executePlan(cp,"myro")

    def executePlan(self, plan, target=None):
        """Sends plan to the specified target for plan execution"""

        if target is None:
            raise ValueError("Must specify target where\
                plan should be executed!")

        elif target=="gazebo":
            # TODO: Execute plan on Gazebo Controls
            pass

        elif target=="myro":
            myro = DeviceProxy("c3/rovers/myro")
            myro.move([0.1, 0.2])
            # Execute plan on myro

        else:
            raise ValueError("%s execution target not-defined!"%target)

    def modifyEnviroment(self, type="add"):
        """Dynamically Modify the current enviroment
        that the rover is located in."""

        #TODO: Add this
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
