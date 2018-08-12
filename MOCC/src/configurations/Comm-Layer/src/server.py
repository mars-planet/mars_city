"""Module for Device based classes"""
import inspect

from flask import Flask, request, jsonify
from flask_restful import Api, Resource
from flasgger import Swagger, swag_from


class Attribute:
    """
    Attribute class to be used as a decorator
    """
    # noinspection PyUnusedLocal
    def __init__(self, fget=None, **kwargs):
        raise NotImplementedError()
        # self._kwargs = kwargs
        # self.name = kwargs.pop("name", None)
        # self.class_name = kwargs.pop("class_name", None)
        # forward = kwargs.pop("forward", False)
        # if forward:
        #     expected = 2 if "label" in kwargs else 1
        #     if len(kwargs) > expected:
        #         raise TypeError(
        #             "Only forwarded attributes support label argument"
        #         )
        #
        # else:
        #     green_mode = kwargs.pop("green_mode", True)
        #     self.read_green_mode = kwargs.pop("read_green_mode", green_mode)
        #     self.write_green_mode = kwargs.pop("write_green_mode",
        #                                        green_mode)
        #
        #     if fget:
        #         if inspect.isroutine(fget):
        #             self.fget = fget
        #             if 'doc' not in kwargs and 'description' not in kwargs:
        #                 if fget.__doc__ is not None:
        #                     kwargs['doc'] = fget.__doc__
        #         kwargs['fget'] = fget
        # super(Attribute, self).__init__(self.name, self.class_name)
        # self.__doc__ = kwargs.get('doc', kwargs.get('description',
        #                                             'TANGO attribute'))
        # if 'dtype' in kwargs:
        #     kwargs['dtype'], kwargs['dformat'] = \
        #         _get_tango_type_format(kwargs['dtype'],
        #                                kwargs.get('dformat'))
        # self.build_from_dict(kwargs)

    def __call__(self):
        raise NotImplementedError()


class Command(object):
    """
    Command decorator. It returns a new `flask_restful.Resource` class
    (not an instance). The returned class is callable, therefore, methods
    decorated with `@Command` can be called directly or through an HTTP POST
    request.
    """
    def __init__(self, func):
        # The type of the class declaring a method decorated with `@Command`
        #  is `None` at declaration time, so we need to declare the
        # `__get__` magic and the `owner_type` property (which must be set)
        # to get around the problem (cf.
        # https://stackoverflow.com/questions/2366713/can-a-python-decorator
        # -of-an-instance-method-access-the-class).
        swagger_specs_dict = {}
        index = 0
        argspec = inspect.getfullargspec(func)
        all_args = argspec.args[1:]
        number_of_args = len(all_args)
        number_of_defaults = 0
        if argspec.defaults is not None:
            number_of_defaults = len(argspec.defaults)

        # Non Default Args
        while index != number_of_args - number_of_defaults:
            parameters = swagger_specs_dict.get('parameters', [])
            parameters.append({
                'name': all_args[index],
                'in': 'path',
                'required': True
            })
            index += 1
            swagger_specs_dict['parameters'] = parameters
        while index != number_of_args:
            parameters = swagger_specs_dict.get('parameters', [])
            parameters.append({
                'name': all_args[index],
                'in': 'path',
                'required': False,
                'default': argspec.defaults[index]
            })
            swagger_specs_dict['parameters'] = parameters
        
        # parser = reqparse.RequestParser()

        # noinspection PyMethodParameters
        class _Command(Resource):
            @swag_from(swagger_specs_dict)
            def post(inner_self):
                params = request.json       
                ret_val = func(self._owner_type(), **params)
                return jsonify(ret_val)

        self.func = func
        self._owner_type = None
        self.resource = _Command
        self.route = func.__name__

    @property
    def owner_type(self):
        return self._owner_type

    @owner_type.setter
    def owner_type(self, val):
        self._owner_type = val

    def __get__(self, obj, type_=None):
        func = self.func.__get__(obj, type_)
        return self.__class__(func)

    def __call__(self, *args, **kwargs):
        return self.func(*args, **kwargs)


class ServerMeta(type):
    """
    Classes using `ServerMeta` as a metaclass will be singletons, i.e. at most
    one instance of the class will exists at any given time.
    """
    _instances: dict = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] =\
                super(ServerMeta, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class ServerBase(Flask, metaclass=ServerMeta):
    """
    Base class for servers.
    Users need to inherit from this class to create their own servers.
    """
    def __init__(self, name):
        """
        `__init__` bounds any `@Command`-decorated methods on child classes to
        HTTP routes, and assigns the correct `owner_type` to them.
        """
        super().__init__(name)
        # noinspection PyTypeChecker
        self.api = Api(self)
        self.swagger = Swagger(self)
        self.config['Swagger'] = {
            'title': 'Mars City',
            'uiversion': 2
        }
        for attr in self.__class__.__dict__.values():
            try:
                self.api.add_resource(attr.resource, f'/{attr.route}')
                if attr.owner_type is None:
                    attr.owner_type = self.__class__
            except AttributeError:
                pass
