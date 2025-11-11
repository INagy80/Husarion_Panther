# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/SystemStatus.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'cpu_percent'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SystemStatus(type):
    """Metaclass of message 'SystemStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('husarion_ugv_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'husarion_ugv_msgs.msg.SystemStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__system_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__system_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__system_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__system_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__system_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SystemStatus(metaclass=Metaclass_SystemStatus):
    """Message class 'SystemStatus'."""

    __slots__ = [
        '_header',
        '_cpu_percent',
        '_cpu_temp',
        '_avg_load_percent',
        '_ram_usage_percent',
        '_disc_usage_percent',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'cpu_percent': 'sequence<float>',
        'cpu_temp': 'float',
        'avg_load_percent': 'float',
        'ram_usage_percent': 'float',
        'disc_usage_percent': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.cpu_percent = array.array('f', kwargs.get('cpu_percent', []))
        self.cpu_temp = kwargs.get('cpu_temp', float())
        self.avg_load_percent = kwargs.get('avg_load_percent', float())
        self.ram_usage_percent = kwargs.get('ram_usage_percent', float())
        self.disc_usage_percent = kwargs.get('disc_usage_percent', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.cpu_percent != other.cpu_percent:
            return False
        if self.cpu_temp != other.cpu_temp:
            return False
        if self.avg_load_percent != other.avg_load_percent:
            return False
        if self.ram_usage_percent != other.ram_usage_percent:
            return False
        if self.disc_usage_percent != other.disc_usage_percent:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def cpu_percent(self):
        """Message field 'cpu_percent'."""
        return self._cpu_percent

    @cpu_percent.setter
    def cpu_percent(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'f', \
                    "The 'cpu_percent' array.array() must have the type code of 'f'"
                self._cpu_percent = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'cpu_percent' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._cpu_percent = array.array('f', value)

    @builtins.property
    def cpu_temp(self):
        """Message field 'cpu_temp'."""
        return self._cpu_temp

    @cpu_temp.setter
    def cpu_temp(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'cpu_temp' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cpu_temp' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cpu_temp = value

    @builtins.property
    def avg_load_percent(self):
        """Message field 'avg_load_percent'."""
        return self._avg_load_percent

    @avg_load_percent.setter
    def avg_load_percent(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'avg_load_percent' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'avg_load_percent' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._avg_load_percent = value

    @builtins.property
    def ram_usage_percent(self):
        """Message field 'ram_usage_percent'."""
        return self._ram_usage_percent

    @ram_usage_percent.setter
    def ram_usage_percent(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'ram_usage_percent' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ram_usage_percent' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ram_usage_percent = value

    @builtins.property
    def disc_usage_percent(self):
        """Message field 'disc_usage_percent'."""
        return self._disc_usage_percent

    @disc_usage_percent.setter
    def disc_usage_percent(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'disc_usage_percent' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'disc_usage_percent' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._disc_usage_percent = value
