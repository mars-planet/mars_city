import collections
from itertools import takewhile
is_tab = '\t'

class Plan():
    """A structured plan generated through EUROPA"""

    def __init__(self, europa_log):
        self.store = {}
        self.objects = []
        self.log = europa_log
        self._structued_rep = None

        # Convert EUROPA log to a structued plan
        lines = europa_log.replace("*************************",
            "").split("\n") # remove europa readability-markers