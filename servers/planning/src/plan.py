import collections
import re
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

class Plan(object):
    """A structured plan generated from a EUROPA temporal network graph"""

    def __init__(self, europa_log):
        self.store = {}
        self.objects = {} # Objects considered in this plan
        self.actions = {} # Objects that require actions to be made
        self.log = europa_log # The raw EUROPA log

        # Get all objects currently in the planning environment
        europa_log = europa_log.replace("*","")
        environment_objects = re.findall(r'Variables(.*?)End Variables',
            europa_log, re.DOTALL)

        # Function which converts the Data Types from PLASMA to a Python format
        def convert_europa_type(variable):
            if "=" in variable:
                res=re.sub(r'=(.*):CLOSED', '=', variable)    
            else:
                res=re.sub(r'.*:CLOSED', '', variable)

            return res

        # Process raw time constrainsts from PLASMA database
        def process_data_format(t):
            t=t.replace("+inf","-1")
            if "{" in t:
                t = t.replace("{","").replace("}","")
                try:
                    t=float(t)
                except:
                    t=str(t)

            elif "[" in t:
                t=t.replace("[","").replace("]","")
                try:
                    t=list(map(float, t.split(",")))
                except:
                    t=list(map(str, t.split(",")))

            return t

        for obj in environment_objects:
            # Build dictionary representation for ease of access
            match = re.search(r'(.*?).name=',obj)
            if match:
                object_name = match.group(1).strip()

                # Clean unnessesary fluff from object attributes        
                attributes = re.findall(r'%s.(.*?)\n'%(object_name),
                    europa_log, re.DOTALL)
                attributes = filter(lambda a: "=" in a, attributes)
                attirbutes = map(lambda a: a.strip(),attributes)

                self.objects[object_name] = {}
                for att in attributes:
                    var = att[:att.find("=")]
                    val = att[att.find("=")+1:]
                    self.objects[object_name][var]=process_data_format(
                        convert_europa_type(val))


        # Extract the Planned Actions to take place along with time constrainsts
        actions_log = re.findall(r'Tokens(.*?)End Tokens', europa_log, re.DOTALL)
        for action in actions_log:
            match = re.search(r'[^\n](.*?)[\.]', action)
            if match:
                actor_name = match.group(1).strip()
                self.actions[actor_name] = []

                # Constrainsts are repsented as a Temporal Network
                start = True
                event_info = []
                last_lower = ""
                for line in action.splitlines():
                    event = {}
                    line = line.strip()
                    if line.startswith("[") or line.startswith("{"):



                        event["lower"]=process_data_format(line)
                        event["upper"]=process_data_format(last_lower)
                        last_lower = line
                        # Save and record past event
                        if "".join(event_info):
                            event["events"] = event_info

                            self.actions[actor_name].append(event)
                        event_info = []
                        event = {}
                    else:
                        if "CLOSED" in line:
                            line = convert_europa_type(line)
                        event_info.append(line)