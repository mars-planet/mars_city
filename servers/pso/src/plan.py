

class Plan(object):
    def __init__(self, europa_log):
        self._structued_rep = None

        # Convert EUROPA log to a structued plan
        plan_string = plan_string.replace("*************************",
            "").split("\n") # remove europa readability-markers

        lines = iter(lines)
        stack = []
        for line in lines:
            indent = len(list(takewhile(is_tab, line)))
            stack[indent:] = [line.lstrip()]
        self._structued_rep = stack

    def __repr__(self):
        return self._structued_rep
