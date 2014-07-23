import collections
from itertools import takewhile
is_tab = '\t'.__eq__

class Plan(collections.MutableMapping):
    """A structured plan generated through EUROPA"""

    def __init__(self, europa_log):
        self.store = dict()

        self._structued_rep = None

        # Convert EUROPA log to a structued plan
        lines = europa_log.replace("*************************",
            "").split("\n") # remove europa readability-markers

        lines = iter(lines)
        stack = []
        for line in lines:
            indent = len(list(takewhile(is_tab, line)))
            stack[indent:] = [line.lstrip()]
        self._structued_rep = stack
        self.update(self._structued_rep)


    def __getitem__(self, key):
        return self.store[self.__keytransform__(key)]

    def __setitem__(self, key, value):
        self.store[self.__keytransform__(key)] = value

    def __delitem__(self, key):
        del self.store[self.__keytransform__(key)]

    def __iter__(self):
        return iter(self.store)

    def __len__(self):
        return len(self.store)

    def __keytransform__(self, key):
        return key
