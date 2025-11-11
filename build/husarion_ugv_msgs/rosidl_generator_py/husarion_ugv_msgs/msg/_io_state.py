# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/IOState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IOState(type):
    """Metaclass of message 'IOState'."""

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
                'husarion_ugv_msgs.msg.IOState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__io_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__io_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__io_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__io_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__io_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IOState(metaclass=Metaclass_IOState):
    """Message class 'IOState'."""

    __slots__ = [
        '_aux_power',
        '_charger_connected',
        '_charger_enabled',
        '_digital_power',
        '_fan',
        '_led_control',
        '_motor_on',
        '_power_button',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'aux_power': 'boolean',
        'charger_connected': 'boolean',
        'charger_enabled': 'boolean',
        'digital_power': 'boolean',
        'fan': 'boolean',
        'led_control': 'boolean',
        'motor_on': 'boolean',
        'power_button': 'boolean',
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
        self.aux_power = kwargs.get('aux_power', bool())
        self.charger_connected = kwargs.get('charger_connected', bool())
        self.charger_enabled = kwargs.get('charger_enabled', bool())
        self.digital_power = kwargs.get('digital_power', bool())
        self.fan = kwargs.get('fan', bool())
        self.led_control = kwargs.get('led_control', bool())
        self.motor_on = kwargs.get('motor_on', bool())
        self.power_button = kwargs.get('power_button', bool())

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
        if self.aux_power != other.aux_power:
            return False
        if self.charger_connected != other.charger_connected:
            return False
        if self.charger_enabled != other.charger_enabled:
            return False
        if self.digital_power != other.digital_power:
            return False
        if self.fan != other.fan:
            return False
        if self.led_control != other.led_control:
            return False
        if self.motor_on != other.motor_on:
            return False
        if self.power_button != other.power_button:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def aux_power(self):
        """Message field 'aux_power'."""
        return self._aux_power

    @aux_power.setter
    def aux_power(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'aux_power' field must be of type 'bool'"
        self._aux_power = value

    @builtins.property
    def charger_connected(self):
        """Message field 'charger_connected'."""
        return self._charger_connected

    @charger_connected.setter
    def charger_connected(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'charger_connected' field must be of type 'bool'"
        self._charger_connected = value

    @builtins.property
    def charger_enabled(self):
        """Message field 'charger_enabled'."""
        return self._charger_enabled

    @charger_enabled.setter
    def charger_enabled(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'charger_enabled' field must be of type 'bool'"
        self._charger_enabled = value

    @builtins.property
    def digital_power(self):
        """Message field 'digital_power'."""
        return self._digital_power

    @digital_power.setter
    def digital_power(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'digital_power' field must be of type 'bool'"
        self._digital_power = value

    @builtins.property
    def fan(self):
        """Message field 'fan'."""
        return self._fan

    @fan.setter
    def fan(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'fan' field must be of type 'bool'"
        self._fan = value

    @builtins.property
    def led_control(self):
        """Message field 'led_control'."""
        return self._led_control

    @led_control.setter
    def led_control(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'led_control' field must be of type 'bool'"
        self._led_control = value

    @builtins.property
    def motor_on(self):
        """Message field 'motor_on'."""
        return self._motor_on

    @motor_on.setter
    def motor_on(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'motor_on' field must be of type 'bool'"
        self._motor_on = value

    @builtins.property
    def power_button(self):
        """Message field 'power_button'."""
        return self._power_button

    @power_button.setter
    def power_button(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'power_button' field must be of type 'bool'"
        self._power_button = value
