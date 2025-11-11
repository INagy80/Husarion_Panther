# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/LEDAnimation.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LEDAnimation(type):
    """Metaclass of message 'LEDAnimation'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'E_STOP': 0,
        'READY': 1,
        'ERROR': 2,
        'NO_ERROR': 3,
        'MANUAL_ACTION': 4,
        'LOW_BATTERY': 5,
        'CRITICAL_BATTERY': 6,
        'CHARGING_BATTERY': 7,
        'BATTERY_CHARGED': 8,
        'CHARGER_INSERTED': 9,
        'BATTERY_NOMINAL': 10,
        'AUTONOMOUS_READY': 11,
        'AUTONOMOUS_ACTION': 12,
        'GOAL_ACHIEVED': 13,
        'BLINKER_LEFT': 14,
        'BLINKER_RIGHT': 15,
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
                'husarion_ugv_msgs.msg.LEDAnimation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__led_animation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__led_animation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__led_animation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__led_animation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__led_animation

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'E_STOP': cls.__constants['E_STOP'],
            'READY': cls.__constants['READY'],
            'ERROR': cls.__constants['ERROR'],
            'NO_ERROR': cls.__constants['NO_ERROR'],
            'MANUAL_ACTION': cls.__constants['MANUAL_ACTION'],
            'LOW_BATTERY': cls.__constants['LOW_BATTERY'],
            'CRITICAL_BATTERY': cls.__constants['CRITICAL_BATTERY'],
            'CHARGING_BATTERY': cls.__constants['CHARGING_BATTERY'],
            'BATTERY_CHARGED': cls.__constants['BATTERY_CHARGED'],
            'CHARGER_INSERTED': cls.__constants['CHARGER_INSERTED'],
            'BATTERY_NOMINAL': cls.__constants['BATTERY_NOMINAL'],
            'AUTONOMOUS_READY': cls.__constants['AUTONOMOUS_READY'],
            'AUTONOMOUS_ACTION': cls.__constants['AUTONOMOUS_ACTION'],
            'GOAL_ACHIEVED': cls.__constants['GOAL_ACHIEVED'],
            'BLINKER_LEFT': cls.__constants['BLINKER_LEFT'],
            'BLINKER_RIGHT': cls.__constants['BLINKER_RIGHT'],
        }

    @property
    def E_STOP(self):
        """Message constant 'E_STOP'."""
        return Metaclass_LEDAnimation.__constants['E_STOP']

    @property
    def READY(self):
        """Message constant 'READY'."""
        return Metaclass_LEDAnimation.__constants['READY']

    @property
    def ERROR(self):
        """Message constant 'ERROR'."""
        return Metaclass_LEDAnimation.__constants['ERROR']

    @property
    def NO_ERROR(self):
        """Message constant 'NO_ERROR'."""
        return Metaclass_LEDAnimation.__constants['NO_ERROR']

    @property
    def MANUAL_ACTION(self):
        """Message constant 'MANUAL_ACTION'."""
        return Metaclass_LEDAnimation.__constants['MANUAL_ACTION']

    @property
    def LOW_BATTERY(self):
        """Message constant 'LOW_BATTERY'."""
        return Metaclass_LEDAnimation.__constants['LOW_BATTERY']

    @property
    def CRITICAL_BATTERY(self):
        """Message constant 'CRITICAL_BATTERY'."""
        return Metaclass_LEDAnimation.__constants['CRITICAL_BATTERY']

    @property
    def CHARGING_BATTERY(self):
        """Message constant 'CHARGING_BATTERY'."""
        return Metaclass_LEDAnimation.__constants['CHARGING_BATTERY']

    @property
    def BATTERY_CHARGED(self):
        """Message constant 'BATTERY_CHARGED'."""
        return Metaclass_LEDAnimation.__constants['BATTERY_CHARGED']

    @property
    def CHARGER_INSERTED(self):
        """Message constant 'CHARGER_INSERTED'."""
        return Metaclass_LEDAnimation.__constants['CHARGER_INSERTED']

    @property
    def BATTERY_NOMINAL(self):
        """Message constant 'BATTERY_NOMINAL'."""
        return Metaclass_LEDAnimation.__constants['BATTERY_NOMINAL']

    @property
    def AUTONOMOUS_READY(self):
        """Message constant 'AUTONOMOUS_READY'."""
        return Metaclass_LEDAnimation.__constants['AUTONOMOUS_READY']

    @property
    def AUTONOMOUS_ACTION(self):
        """Message constant 'AUTONOMOUS_ACTION'."""
        return Metaclass_LEDAnimation.__constants['AUTONOMOUS_ACTION']

    @property
    def GOAL_ACHIEVED(self):
        """Message constant 'GOAL_ACHIEVED'."""
        return Metaclass_LEDAnimation.__constants['GOAL_ACHIEVED']

    @property
    def BLINKER_LEFT(self):
        """Message constant 'BLINKER_LEFT'."""
        return Metaclass_LEDAnimation.__constants['BLINKER_LEFT']

    @property
    def BLINKER_RIGHT(self):
        """Message constant 'BLINKER_RIGHT'."""
        return Metaclass_LEDAnimation.__constants['BLINKER_RIGHT']


class LEDAnimation(metaclass=Metaclass_LEDAnimation):
    """
    Message class 'LEDAnimation'.

    Constants:
      E_STOP
      READY
      ERROR
      NO_ERROR
      MANUAL_ACTION
      LOW_BATTERY
      CRITICAL_BATTERY
      CHARGING_BATTERY
      BATTERY_CHARGED
      CHARGER_INSERTED
      BATTERY_NOMINAL
      AUTONOMOUS_READY
      AUTONOMOUS_ACTION
      GOAL_ACHIEVED
      BLINKER_LEFT
      BLINKER_RIGHT
    """

    __slots__ = [
        '_id',
        '_param',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'id': 'uint16',
        'param': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
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
        self.id = kwargs.get('id', int())
        self.param = kwargs.get('param', str())

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
        if self.id != other.id:
            return False
        if self.param != other.param:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'id' field must be an unsigned integer in [0, 65535]"
        self._id = value

    @builtins.property
    def param(self):
        """Message field 'param'."""
        return self._param

    @param.setter
    def param(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'param' field must be of type 'str'"
        self._param = value
