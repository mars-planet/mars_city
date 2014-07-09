from __future__ import print_function
from pyEUROPA.psengine import stopPSEngine, makePSEngine
import os

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

    def getCurrentPlans():
        return self.europa.planDatabaseToString()

    def createPlans():
        solver = self.europa.createSolver("PlannerConfig.xml");
        solver.configure(0,1000);
        solver.solve(200,200);
