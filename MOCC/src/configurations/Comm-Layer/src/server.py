"""Module for Device based classes"""
import inspect
from types import MethodType, FunctionType
from typing import Union

from flask import Flask, request, jsonify
from flask_restful import Api, Resource
from flasgger import Swagger, swag_from


def _build_swagger_spec(func):
    index = 0
    argspec = inspect.getfullargspec(func)
    all_args = argspec.args[1:]
    number_of_args = len(all_args)
    number_of_defaults = 0
    if argspec.defaults is not None:
        number_of_defaults = len(argspec.defaults)

    swagger_specs_dict = {}
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
    return swagger_specs_dict


class ServerDecorator(object):
    func: Union[MethodType, FunctionType]
    _owner_type: type
    resource: Resource
    route: str


class Attribute(ServerDecorator):
    """
    Attribute decorator. It returns a new `flask_restful.Resource` class
    (not an instance). The returned class is callable, therefore, methods
    decorated with `@Attribute` can be called directly or through an HTTP POST
    request.
    If only the property's getter is needed, use Attribute to decorate the
    method with the @property decoration. If both getter and setter are to be
    accessible through REST, decorate the method with the
    @[property_name].setter decoration.
    """
    def __init__(self, func):
        # The type of the class declaring a method decorated with `@Attribute`
        #  is `None` at declaration time, so we need to declare the
        # `__get__` magic and the `owner_type` property (which must be set)
        # to get around the problem (cf.
        # https://stackoverflow.com/questions/2366713/can-a-python-decorator
        # -of-an-instance-method-access-the-class).

        if not isinstance(func, property):
            raise ValueError(f'{func} must be decorated with @property')

        get_swagger_specs_dict = _build_swagger_spec(func.fget)

        # noinspection PyMethodParameters
        class _AttributeGetInner(Resource):
            @swag_from(get_swagger_specs_dict)
            def get(inner_self):
                ret_val = func.fget(self._owner_type())
                return jsonify(ret_val)

        self.func = func
        self.func_get = func.fget
        self._owner_type = None
        self.resource = _AttributeGetInner
        self.route = f'{func.fget.__name__}'

        self.func_set = None
        self.resource_set = None
        if func.fset is not None:
            set_swagger_specs_dict = _build_swagger_spec(func.fset)

            # noinspection PyMethodParameters
            class _AttributeSetInner(Resource):
                @swag_from(set_swagger_specs_dict)
                def put(inner_self):
                    param = request.json
                    func.fset(self._owner_type(), param)
                    return 200

            self.func_set = func.fset
            self.resource_set = _AttributeSetInner

    @property
    def owner_type(self):
        return self._owner_type

    @owner_type.setter
    def owner_type(self, val):
        self._owner_type = val

    def __get__(self, obj, type_=None):
        ret_val = self.func.__get__(obj, type_)
        return ret_val

    def __call__(self, *args, **kwargs):
        self.func_set(*args, **kwargs)


class Command(ServerDecorator):
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

        swagger_specs_dict = _build_swagger_spec(func)

        # noinspection PyMethodParameters
        class _CommandInner(Resource):
            @swag_from(swagger_specs_dict)
            def post(inner_self):
                params = request.json       
                ret_val = func(self._owner_type(), **params)
                return jsonify(ret_val)

        self.func = func
        self._owner_type = None
        self.resource = _CommandInner
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
        self.commands = []
        self.attributes = []
        for attr in self.__class__.__dict__.values():
            if isinstance(attr, ServerDecorator):
                # noinspection PyTypeChecker
                self.api.add_resource(attr.resource, f'/{attr.route}')
                if attr.owner_type is None:
                    attr.owner_type = self.__class__
            if isinstance(attr, Command):
                self.commands.append(attr)
            elif isinstance(attr, Attribute):
                if attr.resource_set is not None:
                    # noinspection PyTypeChecker
                    self.api.add_resource(attr.resource_set, f'/{attr.route}')
                self.attributes.append(attr)
