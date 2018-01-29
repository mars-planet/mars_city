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