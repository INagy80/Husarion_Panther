# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/RobotDriverState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotDriverState(type):
    """Metaclass of message 'RobotDriverState'."""

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
                'husarion_ugv_msgs.msg.RobotDriverState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_driver_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_driver_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_driver_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_driver_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_driver_state

            from husarion_ugv_msgs.msg import DriverStateNamed
            if DriverStateNamed.__class__._TYPE_SUPPORT is None:
                DriverStateNamed.__class__.__import_type_support__()

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


class RobotDriverState(metaclass=Metaclass_RobotDriverState):
    """Message class 'RobotDriverState'."""

    __slots__ = [
        '_header',
        '_driver_states',
        '_error',
        '_write_pdo_cmds_error',
        '_read_pdo_motor_states_error',
        '_read_pdo_driver_state_error',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'driver_states': 'sequence<husarion_ugv_msgs/DriverStateNamed>',
        'error': 'boolean',
        'write_pdo_cmds_error': 'boolean',
        'read_pdo_motor_states_error': 'boolean',
        'read_pdo_driver_state_error': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['husarion_ugv_msgs', 'msg'], 'DriverStateNamed')),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.driver_states = kwargs.get('driver_states', [])
        self.error = kwargs.get('error', bool())
        self.write_pdo_cmds_error = kwargs.get('write_pdo_cmds_error', bool())
        self.read_pdo_motor_states_error = kwargs.get('read_pdo_motor_states_error', bool())
        self.read_pdo_driver_state_error = kwargs.get('read_pdo_driver_state_error', bool())

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
        if self.driver_states != other.driver_states:
            return False
        if self.error != other.error:
            return False
        if self.write_pdo_cmds_error != other.write_pdo_cmds_error:
            return False
        if self.read_pdo_motor_states_error != other.read_pdo_motor_states_error:
            return False
        if self.read_pdo_driver_state_error != other.read_pdo_driver_state_error:
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
    def driver_states(self):
        """Message field 'driver_states'."""
        return self._driver_states

    @driver_states.setter
    def driver_states(self, value):
        if self._check_fields:
            from husarion_ugv_msgs.msg import DriverStateNamed
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
                 all(isinstance(v, DriverStateNamed) for v in value) and
                 True), \
                "The 'driver_states' field must be a set or sequence and each value of type 'DriverStateNamed'"
        self._driver_states = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'error' field must be of type 'bool'"
        self._error = value

    @builtins.property
    def write_pdo_cmds_error(self):
        """Message field 'write_pdo_cmds_error'."""
        return self._write_pdo_cmds_error

    @write_pdo_cmds_error.setter
    def write_pdo_cmds_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'write_pdo_cmds_error' field must be of type 'bool'"
        self._write_pdo_cmds_error = value

    @builtins.property
    def read_pdo_motor_states_error(self):
        """Message field 'read_pdo_motor_states_error'."""
        return self._read_pdo_motor_states_error

    @read_pdo_motor_states_error.setter
    def read_pdo_motor_states_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'read_pdo_motor_states_error' field must be of type 'bool'"
        self._read_pdo_motor_states_error = value

    @builtins.property
    def read_pdo_driver_state_error(self):
        """Message field 'read_pdo_driver_state_error'."""
        return self._read_pdo_driver_state_error

    @read_pdo_driver_state_error.setter
    def read_pdo_driver_state_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'read_pdo_driver_state_error' field must be of type 'bool'"
        self._read_pdo_driver_state_error = value
