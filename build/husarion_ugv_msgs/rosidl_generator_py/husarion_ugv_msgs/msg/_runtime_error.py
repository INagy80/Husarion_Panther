# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/RuntimeError.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RuntimeError(type):
    """Metaclass of message 'RuntimeError'."""

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
                'husarion_ugv_msgs.msg.RuntimeError')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__runtime_error
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__runtime_error
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__runtime_error
            cls._TYPE_SUPPORT = module.type_support_msg__msg__runtime_error
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__runtime_error

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RuntimeError(metaclass=Metaclass_RuntimeError):
    """Message class 'RuntimeError'."""

    __slots__ = [
        '_amps_limit_active',
        '_motor_stall',
        '_loop_error',
        '_safety_stop_active',
        '_forward_limit_triggered',
        '_reverse_limit_triggered',
        '_amps_trigger_activated',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'amps_limit_active': 'boolean',
        'motor_stall': 'boolean',
        'loop_error': 'boolean',
        'safety_stop_active': 'boolean',
        'forward_limit_triggered': 'boolean',
        'reverse_limit_triggered': 'boolean',
        'amps_trigger_activated': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.amps_limit_active = kwargs.get('amps_limit_active', bool())
        self.motor_stall = kwargs.get('motor_stall', bool())
        self.loop_error = kwargs.get('loop_error', bool())
        self.safety_stop_active = kwargs.get('safety_stop_active', bool())
        self.forward_limit_triggered = kwargs.get('forward_limit_triggered', bool())
        self.reverse_limit_triggered = kwargs.get('reverse_limit_triggered', bool())
        self.amps_trigger_activated = kwargs.get('amps_trigger_activated', bool())

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
        if self.amps_limit_active != other.amps_limit_active:
            return False
        if self.motor_stall != other.motor_stall:
            return False
        if self.loop_error != other.loop_error:
            return False
        if self.safety_stop_active != other.safety_stop_active:
            return False
        if self.forward_limit_triggered != other.forward_limit_triggered:
            return False
        if self.reverse_limit_triggered != other.reverse_limit_triggered:
            return False
        if self.amps_trigger_activated != other.amps_trigger_activated:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def amps_limit_active(self):
        """Message field 'amps_limit_active'."""
        return self._amps_limit_active

    @amps_limit_active.setter
    def amps_limit_active(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'amps_limit_active' field must be of type 'bool'"
        self._amps_limit_active = value

    @builtins.property
    def motor_stall(self):
        """Message field 'motor_stall'."""
        return self._motor_stall

    @motor_stall.setter
    def motor_stall(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'motor_stall' field must be of type 'bool'"
        self._motor_stall = value

    @builtins.property
    def loop_error(self):
        """Message field 'loop_error'."""
        return self._loop_error

    @loop_error.setter
    def loop_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'loop_error' field must be of type 'bool'"
        self._loop_error = value

    @builtins.property
    def safety_stop_active(self):
        """Message field 'safety_stop_active'."""
        return self._safety_stop_active

    @safety_stop_active.setter
    def safety_stop_active(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'safety_stop_active' field must be of type 'bool'"
        self._safety_stop_active = value

    @builtins.property
    def forward_limit_triggered(self):
        """Message field 'forward_limit_triggered'."""
        return self._forward_limit_triggered

    @forward_limit_triggered.setter
    def forward_limit_triggered(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'forward_limit_triggered' field must be of type 'bool'"
        self._forward_limit_triggered = value

    @builtins.property
    def reverse_limit_triggered(self):
        """Message field 'reverse_limit_triggered'."""
        return self._reverse_limit_triggered

    @reverse_limit_triggered.setter
    def reverse_limit_triggered(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'reverse_limit_triggered' field must be of type 'bool'"
        self._reverse_limit_triggered = value

    @builtins.property
    def amps_trigger_activated(self):
        """Message field 'amps_trigger_activated'."""
        return self._amps_trigger_activated

    @amps_trigger_activated.setter
    def amps_trigger_activated(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'amps_trigger_activated' field must be of type 'bool'"
        self._amps_trigger_activated = value
