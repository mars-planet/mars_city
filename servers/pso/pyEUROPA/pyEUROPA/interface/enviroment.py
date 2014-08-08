from pyEUROPA.engine import makePSEngine, stopPSEngine

class Enviroment(object):
    """Planning environment"""

    _objects = []
    _locations = {}

    def set_rover():


    def set_rover_goal():
        pass

    def add_location(self, name, position):
        """Locations are places of interest for the rover"""

        if name in self._locations:
            raise ValueError("Location already exists in environment!")
        self._locations[name]=instance

    def remove_location(self, name):
        del self._locations[name]

    def get_plan():
        pass

    def go(self):

        # Convert Python to EUROPA's NDDL
        self._is_alive = True
        self._psengine = makePSEngine()

        # Load actor's initial enviroment
        errors = self._psengine.executeScript("nddl",enviroment, True);

        if (errors!=""):
            raise ValueError("Failed loading inital enviroment:"+errors);


    def kill(self):
        self._is_alive = False
        stopPSEngine(self._psengine)
