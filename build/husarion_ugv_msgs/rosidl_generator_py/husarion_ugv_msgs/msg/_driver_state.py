# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/DriverState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DriverState(type):
    """Metaclass of message 'DriverState'."""

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
                'husarion_ugv_msgs.msg.DriverState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__driver_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__driver_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__driver_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__driver_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__driver_state

            from husarion_ugv_msgs.msg import FaultFlag
            if FaultFlag.__class__._TYPE_SUPPORT is None:
                FaultFlag.__class__.__import_type_support__()

            from husarion_ugv_msgs.msg import RuntimeError
            if RuntimeError.__class__._TYPE_SUPPORT is None:
                RuntimeError.__class__.__import_type_support__()

            from husarion_ugv_msgs.msg import ScriptFlag
            if ScriptFlag.__class__._TYPE_SUPPORT is None:
                ScriptFlag.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DriverState(metaclass=Metaclass_DriverState):
    """Message class 'DriverState'."""

    __slots__ = [
        '_voltage',
        '_current',
        '_temperature',
        '_heatsink_temperature',
        '_fault_flag',
        '_script_flag',
        '_channel_1_motor_runtime_error',
        '_channel_2_motor_runtime_error',
        '_motor_states_data_timed_out',
        '_driver_state_data_timed_out',
        '_can_error',
        '_heartbeat_timeout',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'voltage': 'float',
        'current': 'float',
        'temperature': 'float',
        'heatsink_temperature': 'float',
        'fault_flag': 'husarion_ugv_msgs/FaultFlag',
        'script_flag': 'husarion_ugv_msgs/ScriptFlag',
        'channel_1_motor_runtime_error': 'husarion_ugv_msgs/RuntimeError',
        'channel_2_motor_runtime_error': 'husarion_ugv_msgs/RuntimeError',
        'motor_states_data_timed_out': 'boolean',
        'driver_state_data_timed_out': 'boolean',
        'can_error': 'boolean',
        'heartbeat_timeout': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['husarion_ugv_msgs', 'msg'], 'FaultFlag'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['husarion_ugv_msgs', 'msg'], 'ScriptFlag'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['husarion_ugv_msgs', 'msg'], 'RuntimeError'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['husarion_ugv_msgs', 'msg'], 'RuntimeError'),  # noqa: E501
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
        self.voltage = kwargs.get('voltage', float())
        self.current = kwargs.get('current', float())
        self.temperature = kwargs.get('temperature', float())
        self.heatsink_temperature = kwargs.get('heatsink_temperature', float())
        from husarion_ugv_msgs.msg import FaultFlag
        self.fault_flag = kwargs.get('fault_flag', FaultFlag())
        from husarion_ugv_msgs.msg import ScriptFlag
        self.script_flag = kwargs.get('script_flag', ScriptFlag())
        from husarion_ugv_msgs.msg import RuntimeError
        self.channel_1_motor_runtime_error = kwargs.get('channel_1_motor_runtime_error', RuntimeError())
        from husarion_ugv_msgs.msg import RuntimeError
        self.channel_2_motor_runtime_error = kwargs.get('channel_2_motor_runtime_error', RuntimeError())
        self.motor_states_data_timed_out = kwargs.get('motor_states_data_timed_out', bool())
        self.driver_state_data_timed_out = kwargs.get('driver_state_data_timed_out', bool())
        self.can_error = kwargs.get('can_error', bool())
        self.heartbeat_timeout = kwargs.get('heartbeat_timeout', bool())

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
        if self.voltage != other.voltage:
            return False
        if self.current != other.current:
            return False
        if self.temperature != other.temperature:
            return False
        if self.heatsink_temperature != other.heatsink_temperature:
            return False
        if self.fault_flag != other.fault_flag:
            return False
        if self.script_flag != other.script_flag:
            return False
        if self.channel_1_motor_runtime_error != other.channel_1_motor_runtime_error:
            return False
        if self.channel_2_motor_runtime_error != other.channel_2_motor_runtime_error:
            return False
        if self.motor_states_data_timed_out != other.motor_states_data_timed_out:
            return False
        if self.driver_state_data_timed_out != other.driver_state_data_timed_out:
            return False
        if self.can_error != other.can_error:
            return False
        if self.heartbeat_timeout != other.heartbeat_timeout:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._voltage = value

    @builtins.property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._temperature = value

    @builtins.property
    def heatsink_temperature(self):
        """Message field 'heatsink_temperature'."""
        return self._heatsink_temperature

    @heatsink_temperature.setter
    def heatsink_temperature(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'heatsink_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heatsink_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heatsink_temperature = value

    @builtins.property
    def fault_flag(self):
        """Message field 'fault_flag'."""
        return self._fault_flag

    @fault_flag.setter
    def fault_flag(self, value):
        if self._check_fields:
            from husarion_ugv_msgs.msg import FaultFlag
            assert \
                isinstance(value, FaultFlag), \
                "The 'fault_flag' field must be a sub message of type 'FaultFlag'"
        self._fault_flag = value

    @builtins.property
    def script_flag(self):
        """Message field 'script_flag'."""
        return self._script_flag

    @script_flag.setter
    def script_flag(self, value):
        if self._check_fields:
            from husarion_ugv_msgs.msg import ScriptFlag
            assert \
                isinstance(value, ScriptFlag), \
                "The 'script_flag' field must be a sub message of type 'ScriptFlag'"
        self._script_flag = value

    @builtins.property
    def channel_1_motor_runtime_error(self):
        """Message field 'channel_1_motor_runtime_error'."""
        return self._channel_1_motor_runtime_error

    @channel_1_motor_runtime_error.setter
    def channel_1_motor_runtime_error(self, value):
        if self._check_fields:
            from husarion_ugv_msgs.msg import RuntimeError
            assert \
                isinstance(value, RuntimeError), \
                "The 'channel_1_motor_runtime_error' field must be a sub message of type 'RuntimeError'"
        self._channel_1_motor_runtime_error = value

    @builtins.property
    def channel_2_motor_runtime_error(self):
        """Message field 'channel_2_motor_runtime_error'."""
        return self._channel_2_motor_runtime_error

    @channel_2_motor_runtime_error.setter
    def channel_2_motor_runtime_error(self, value):
        if self._check_fields:
            from husarion_ugv_msgs.msg import RuntimeError
            assert \
                isinstance(value, RuntimeError), \
                "The 'channel_2_motor_runtime_error' field must be a sub message of type 'RuntimeError'"
        self._channel_2_motor_runtime_error = value

    @builtins.property
    def motor_states_data_timed_out(self):
        """Message field 'motor_states_data_timed_out'."""
        return self._motor_states_data_timed_out

    @motor_states_data_timed_out.setter
    def motor_states_data_timed_out(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'motor_states_data_timed_out' field must be of type 'bool'"
        self._motor_states_data_timed_out = value

    @builtins.property
    def driver_state_data_timed_out(self):
        """Message field 'driver_state_data_timed_out'."""
        return self._driver_state_data_timed_out

    @driver_state_data_timed_out.setter
    def driver_state_data_timed_out(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'driver_state_data_timed_out' field must be of type 'bool'"
        self._driver_state_data_timed_out = value

    @builtins.property
    def can_error(self):
        """Message field 'can_error'."""
        return self._can_error

    @can_error.setter
    def can_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'can_error' field must be of type 'bool'"
        self._can_error = value

    @builtins.property
    def heartbeat_timeout(self):
        """Message field 'heartbeat_timeout'."""
        return self._heartbeat_timeout

    @heartbeat_timeout.setter
    def heartbeat_timeout(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'heartbeat_timeout' field must be of type 'bool'"
        self._heartbeat_timeout = value
