from __future__ import print_function

from pyEUROPA.psengine import makePSEngine, stopPSEngine

# Launch & connect to EUROPA
europa = makePSEngine()
europa.start()
europa.executeScript("nddl","SimpleRover-model.nddl",True)

# Run solver
plannerConfig = "PlannerConfig.xml"
startHorizon, endHorizon= 0, 100

solver = europa.createSolver(plannerConfig)
solver.configure(startHorizon,endHorizon)

maxSteps = 1000
i = 0
while(not solver.isExhausted() and not solver.isTimedOut() and i<maxSteps):
    solver.step()

    if (solver.isConstraintConsistent()):
        flaws = solver.getFlaws();
        if (solver.getFlaws().size() == 0):
            break
    else:
        print("Solver is not constraint consistent at step "+str(i))

    i = solver.getStepCount()

if (solver.isExhausted()):
    print("Solver was exhausted after " + str(i) + " steps");     
elif (solver.isTimedOut()):
    print("Solver timed out after " + str(i) + " steps");
else:
    print("Main","Solver finished after " + str(i) + " steps");

# Shuts down all PSEngine instances
stopPSEngine()