"""Module for Device based classes"""
class Device:
    """
    Device Class for the high-level API
    All device specific classes should inherit from this class.
    """

    def __init__(self):
        self.attr_list = set()
        self.command_list = set()
        self.status = ""
        self.name = ""

    def add_attribute(self, attr: str, r_meth=None, w_meth=None, is_allo_meth=None):
        """
        Add a new attribute to the device attribute list
        :param Attr attr: the new attribute to add
        """
        self.attr_list.add(attr)

    def add_command(self, cmd: str, device_level):
        """
        Add a new command to the device command list
        :param str cmd: the new command to add
        :param bool device_level: Set this flag to true
        if the command must be added for only this device
        """
        self.command_list.add(cmd)
        if device_level is False:
            raise NotImplementedError()

    def append_status(self, status: str, new_line=False):
        """
        Appends a string to the device status
        :param str status: the string to be appended the device status.
        :param bool new_line: If true, appends new line character
        before the string. Default is False.
        """
        if new_line:
            status = "\n" + status
        self.status += status

    def check_command_exists(self, cmd_name: str) -> bool:
        """
        This method checks if a command is supported by the device.
        It does not need any input value and throws an exception if the command is not defined
        or needs an input value
        :param str cmd_name: the command name to check
        """
        return cmd_name in self.command_list

    def debug_stream(self, msg: str, *args):
        """
        Sends the given message to the debug stream.
        :param msg:
        :param args:
        """
        raise NotImplementedError()

    def delete_device(self):
        """
        Delete the device.
        """
        raise NotImplementedError()

    def error_stream(self, msg: str, *args):
        """
        Sends the given message to the error stream.
        :param str msg: The message to send.
        :param args:
        """
        raise NotImplementedError()

    def fatal_stream(self, msg: str, *args):
        """
        Sends the given message to the fatal stream.
        :param str msg: The message to send.
        :param args:
        """
        raise NotImplementedError()

    def get_attr_min_poll_period(self):
        """
        Returns the min attribute poll period
        :return seq: the min attribute poll period
        """
        raise NotImplementedError()

    def get_attr_poll_ring_depth(self, attr_name):
        """
        Returns the attribute poll ring depth
        :param str attr_name: the attribute name
        :return int: the attribute poll ring depth
        """
        raise NotImplementedError()

    def get_attribute_poll_period(self, attr_name):
        """
        Returns the attribute polling period (ms) or 0 if the attribute is not polled.
        :param str attr_name: the attribute name
        :return int: the attribute poll period (ms) or 0 if not polled.
        """
        raise NotImplementedError()

    def get_cmd_min_poll_period(self):
        """
        Returns the min command poll period
        :return seq<str>: the min command poll period
        """
        raise NotImplementedError()

    def get_cmd_poll_ring_depth(self, cmd_name):
        """
        Returns the command poll ring depth
        :param str cmd_name: the name of the command
        :return int: the command poll ring depth
        """
        raise NotImplementedError()

    def get_command_poll_period(self, cmd_name):
        """
        Returns the command polling period (ms) or 0 if the command is not polled.
        :param str cmd_name: the name of the command
        :return int: the command polling period (ms) or 0 if the command is not polled.
        """
        raise NotImplementedError()

    def get_device_attr(self):
        """
        Get device multi attribute object
        :return MultiAttribute: the device's multi attribute object
        """
        raise NotImplementedError()

    def get_exported_flag(self):
        """
        Returns the state of the exported flag
        :return bool: the state of the exported flag
        """
        raise NotImplementedError()

    def get_logger(self):
        """
        Returns the logger object of this device
        :return Logger: the Logger object for this device
        """
        raise NotImplementedError()

    def get_min_poll_period(self):
        """
        Returns the min poll period
        :return int: Returns the min poll period
        """
        raise NotImplementedError()

    def get_name(self):
        """
        Get a COPY of the device name
        :return str: the device name
        """
        return self.name

    def get_non_auto_polled_attr(self):
        """
        Returns a COPY of the list of non automatic polled attributes
        :return seq<str>: a COPY of the list of non automatic polled attributes
        """
        raise NotImplementedError()

    def get_non_auto_polled_cmd(self):
        """
        Returns a COPY of the list of non automatic polled commands
        :return seq<str>: a COPY of the list of non automatic polled commands
        """
        raise NotImplementedError()

    def get_poll_old_factor(self):
        """
        Returns the poll old factor
        :return int: the poll old factor
        """
        raise NotImplementedError()

    def get_poll_ring_depth(self):
        """
        Returns the poll ring depth
        :return int: the poll ring depth
        """
        raise NotImplementedError()

    def get_polled_attr(self):
        """
        Returns a COPY of the list of polled attributes
        :return seq<str>: a COPY of the list of polled attributes
        """
        raise NotImplementedError()

    def get_polled_cmd(self):
        """
        Returns a COPY of the list of polled commands
        :return seq<str>: a COPY of the list of polled commands
        """
        raise NotImplementedError()

    def get_prev_state(self):
        """
        Get a COPY of the device’s previous state.
        :return DevState: the device's previous state
        """
        raise NotImplementedError()

    def get_state(self):
        """
        Get a COPY of the device's state
        :return DevState: Current device state
        """
        raise NotImplementedError()

    def get_status(self):
        """
        Get a COPY of the device status.
        :return str: the device status
        """
        return self.status

    def info_stream(self, msg):
        """
        Sends the given message to the info stream.
        :param msg str: the given message to the info
        """
        raise NotImplementedError()

    def init_device(self):
        """"""
        raise NotImplementedError()

    def initialize_dynamic_attributes(self):
        """
        Method executed at initializion phase to create dynamic attributes. 
        Default implementation does nothing. Overwrite when necessary.
        """
        raise NotImplementedError()

    def is_device_locked(self):
        """
        Returns if this device is locked by a client.
        :return bool: True if it is locked or False otherwise
        """
        raise NotImplementedError()

    def is_polled(self):
        """
        Returns if it is polled
        :return bool: True if it is polled or False otherwise
        """
    
    def is_there_subscriber(self, att_name, event_type):
        """
        Check if there is subscriber(s) listening for the event.
        :param str att_name: the attribute name
        :param EventType event_type: the event type
        :return bool: True if there is at least one listener or False otherwise
        """
        raise NotImplementedError()

    def push_archive_event(self, attr_name, data=None, str_data=None, except=None, dim_x=None, dim_y=None, time_stamp=None, quality=None):
        """
        Push an archive event for the given attribute name. The event is pushed to the notification daemon.
        :param str attr_name:	(str) attribute name
        :param Object data:	the data to be sent as attribute event data. Data must be compatible with the attribute type and format. for SPECTRUM and IMAGE attributes, data can be any type of sequence of elements compatible with the attribute type
        :param str str_data:	(str) special variation for DevEncoded data type. In this case ‘data’ must be a str or an object with the buffer interface.
        :param DevFailed except:	(DevFailed) Instead of data, you may want to send an exception.
        :param int dim_x:	(int) the attribute x length. Default value is 1
        :param int dim_y:	(int) the attribute y length. Default value is 0
        :param double time_stamp:	(double) the time stamp
        :param AttrQuality quality:	(AttrQuality) the attribute quality factor
        """
        raise NotImplementedError()
    
    def push_att_conf_event(self, attr):
        """
        Push an attribute configuration event
        :param Attribute attr: the attribute for which the configuration event will be sent
        """
        raise NotImplementedError()

    def push_change_event(self, attr_name, data=None, str_data=None, except=None, dim_x=None, dim_y=None, time_stamp=None, quality=None):
        """
        Push a change event for the given attribute name. The event is pushed to the notification daemon.
        :param str attr_name: (str) attribute name
        :param Object data:	the data to be sent as attribute event data. Data must be compatible with the attribute type and format. for SPECTRUM and IMAGE attributes, data can be any type of sequence of elements compatible with the attribute type
        :param str str_data: (str) special variation for DevEncoded data type. In this case ‘data’ must be a str or an object with the buffer interface.
        :param DevFailed except: (DevFailed) Instead of data, you may want to send an exception.
        :param int dim_x: (int) the attribute x length. Default value is 1
        :param int dim_y: (int) the attribute y length. Default value is 0
        :param double time_stamp:	(double) the time stamp
        :param AttrQuality quality: (AttrQuality) the attribute quality factor
        """
        raise NotImplementedError()

    def push_data_ready_event(self, attr_name, counter=0):
        """
        Push a data ready event for the given attribute name. The event is pushed to the notification daemon.
        The method needs only the attribue name and an optional “counter” which will be passed unchanged within the event.
        :param str attr_name: the attribute name
        :param int counter: the user counter
        """
        raise NotImplementedError()

    def push_event(self, attr_name, filt_names=None, filt_vals=None, data=None, str_data=None, dim_x=None, dim_y=None, time_stamp=None, quality=None):
        """
        :param str attr_name: (str) attribute name
        :param sequence<str> filt_names: (sequence<str>) the filterable fields name
        :param sequence<double> filt_vals: (sequence<double>) the filterable fields value
        :param Object data:	the data to be sent as attribute event data. Data must be compatible with the attribute type and format. for SPECTRUM and IMAGE attributes, data can be any type of sequence of elements compatible with the attribute type
        :param str str_data: (str) special variation for DevEncoded data type. In this case ‘data’ must be a str or an object with the buffer interface.
        :param int dim_x: (int) the attribute x length. Default value is 1
        :param int dim_y: (int) the attribute y length. Default value is 0
        :param double time_stamp: (double) the time stamp
        :param AttrQuality quality: (AttrQuality) the attribute quality factor
        """
        raise NotImplementedError()

    def push_pipe_event(self, blob):
        """
        Push a pipe event
        :param object blob: the blob which pipe event will be send
        """
        raise NotImplementedError()

    def register_signal(self, signo):
        """
        Register a signal. Register this device as device to be informed when signal signo is sent to to the device server process.
        :param int signo: signal identifier
        """
        raise NotImplementedError()

    def remove_attribute(self, attr_name):
        """
        Remove one attribute from the device ttribute list
        :param str attr_name: the attribute name
        """
        # TO-DO: Raise DevFailed if `remove` raises KeyError
        self.attr_list.remove(attr_name)

    def remove_command(self,cmd_name):
        """
        Remove one command from the device command list.
        :param str cmd_name: the command name to remove
        """
        # TO-DO: Raise DevFailed if `remove` raises KeyError
        self.command_list.remove(cmd_name)

    def run_server(self, args=None, **kwargs):
        """
        Run the class as a device server
        The difference is that the device class and server name are automatically given
        """
        raise NotImplementedError()

    def set_archive_event(self, attr_name, implemented, detect=True):
        """
        Set an implemented flag for the attribute to indicate that the server fires archive events manually, without the polling to be started. 
        If the detect parameter is set to true, the criteria specified for the archive event are verified 
        and the event is only pushed if they are fullfilled. 
        If detect is set to false the event is fired without any value checking!
        :param str attr_name: (str) attribute name
        :param bool implemented: (bool) True when the server fires change events manually.
        :param bool detect: (bool) Triggers the verification of the change event properties when set to true. Default value is true.
        """
        raise NotImplementedError()

    def set_change_event(self, attr_name, implemented, detect=True):
        """
        Set an implemented flag for the attribute to indicate that the server fires change events manually, without the polling to be started. 
        If the detect parameter is set to true, the criteria specified for the change event are verified and the 
        event is only pushed if they are fullfilled. If detect is set to false the event is fired without any value checking!
        :param str attr_name: (str) attribute name
        :param bool implemented: (bool) True when the server fires change events manually.
        :param bool detect: (bool) Triggers the verification of the change event properties when set to true. Default value is true.
        """
        raise NotImplementedError()

    def set_status(self, new_status):
        """
        Set device status
        :param str new_status: (str) the new device status
        """
        self.status = new_status

    def signal_handler(self, signo):
        """
        Signal handler. The method executed when the signal arrived in the device server process. 
        This method is defined as virtual and then, can be redefined following device needs.
        :param int signo: the signal number 
        """
        raise NotImplementedError()

    def stop_polling(self, with_db_upd):
        """
        Stop all polling for a device. if the device is polled, call this method before deleting it.
        :param bool with_db_upd: (bool) Is it necessary to update db ?
        """
        raise NotImplementedError()

    def unregister_signal(self, signo):
        """
        Unregister a signal. Unregister this device as device to be informed when signal signo is 
        sent to to the device server process
        :param int signo: the signal identifier
        """
        raise NotImplementedError()

    def warn_stream(self, msg, *args):
        """
        Sends the given message to the tango warn stream.
        :param str msg: (str) the message to be sent to the warn stream
        """
        raise NotImplementedError()

    def write_attr_hardware(self, attr_list):
        """
        Write the hardware for attributes. Default method to implement an action necessary on a device to 
        write the hardware involved in a a write attribute. This method must be redefined in sub-classes in order 
        to support writable attribute
        :param sequence<int> attr_list: list of indices in the device object attribute vector of an attribute to be written.
        """
        raise NotImplementedError()


class attribute:
    """
    Attribute class to be used as a decorator
    """
    def __init__(self, fget=None, **kwargs):
        self._kwargs = kwargs
        self.name = kwargs.pop("name", None)
        self.class_name = kwargs.pop("class_name", None)
        forward = kwargs.pop("forward", False)
        if forward:
            expected = 2 if "label" in kwargs else 1
            if len(kwargs) > expected:
                raise TypeError("Only forwarded attributes support label argument")

        else:
            green_mode = kwargs.pop("green_mode", True)
            self.read_green_mode = kwargs.pop("read_green_mode", green_mode)
            self.write_green_mode = kwargs.pop("write_green_mode", green_mode)

            if fget:
                if inspect.isroutine(fget):
                    self.fget = fget
                    if 'doc' not in kwargs and 'description' not in kwargs:
                        if fget.__doc__ is not None:
                            kwargs['doc'] = fget.__doc__
                kwargs['fget'] = fget
        super(attribute, self).__init__(self.name, class_name)
        self.__doc__ = kwargs.get('doc', kwargs.get('description',
                                                    'TANGO attribute'))
        if 'dtype' in kwargs:
            kwargs['dtype'], kwargs['dformat'] = \
                _get_tango_type_format(kwargs['dtype'], kwargs.get('dformat'))
        self.build_from_dict(kwargs)

    def __call__(self):
        raise NotImplementedError()