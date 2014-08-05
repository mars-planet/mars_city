import collections
from itertools import takewhile

def build_tree(log):
    is_tab = '\t'.__eq__
    log = log.replace("   ","\t").replace("*","") 
    lines = iter(log.split("\n"))
    tree = []
    stack = []
    for line in lines:
        indent = len(list(takewhile(is_tab, line)))
        stack[indent:] = [line.lstrip()]
        tree.append(list(stack))
    return tree

class Plan():
    """A structured plan generated through EUROPA"""

    def __init__(self, europa_log):
        self.store = {}
        self.objects = []
        self.log = europa_log # Keep the raw EUROPA log
        self._structued_rep = None

        # Specific recorded actions
        self.rover_move = []

        # Iterate through log and extract actionable plans
        tree = build_tree(self.log)
        for level in tree:
            for node in level:

                # Rover movement actions
                if "Rover.Go" in node:
                    self.rover_move.append(node)