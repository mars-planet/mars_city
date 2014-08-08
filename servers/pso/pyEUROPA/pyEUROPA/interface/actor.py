from pyEUROPA.engine import makePSEngine, stopPSEngine

class Actor(object):
    """An actor is an instance in an EUROPA enviroment doing the planning"""

    _is_alive = False
    _psengine = None

    def __init__(self, *args):
        self._args = args

    def bind(self, event, callback):
        """Bind some event with the actor"""
        pass