from inspect import getsource

def parse_nddl_class(python_object):
    py_class = python_object.__class__
    source = getsource(py_class)
    print source


class Rover(object):
    pass

parse_nddl_class(Rover())