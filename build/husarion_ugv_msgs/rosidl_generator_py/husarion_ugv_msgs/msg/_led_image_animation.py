# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
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


class Metaclass_LEDImageAnimation(type):
    """Metaclass of message 'LEDImageAnimation'."""

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
                'husarion_ugv_msgs.msg.LEDImageAnimation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__led_image_animation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__led_image_animation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__led_image_animation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__led_image_animation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__led_image_animation

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LEDImageAnimation(metaclass=Metaclass_LEDImageAnimation):
    """Message class 'LEDImageAnimation'."""

    __slots__ = [
        '_image',
        '_duration',
        '_brightness',
        '_repeat',
        '_color',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'image': 'string',
        'duration': 'float',
        'brightness': 'float',
        'repeat': 'uint8',
        'color': 'uint32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
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
        self.image = kwargs.get('image', str())
        self.duration = kwargs.get('duration', float())
        self.brightness = kwargs.get('brightness', float())
        self.repeat = kwargs.get('repeat', int())
        self.color = kwargs.get('color', int())

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
        if self.image != other.image:
            return False
        if self.duration != other.duration:
            return False
        if self.brightness != other.brightness:
            return False
        if self.repeat != other.repeat:
            return False
        if self.color != other.color:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def image(self):
        """Message field 'image'."""
        return self._image

    @image.setter
    def image(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'image' field must be of type 'str'"
        self._image = value

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'duration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'duration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._duration = value

    @builtins.property
    def brightness(self):
        """Message field 'brightness'."""
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'brightness' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'brightness' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._brightness = value

    @builtins.property
    def repeat(self):
        """Message field 'repeat'."""
        return self._repeat

    @repeat.setter
    def repeat(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'repeat' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'repeat' field must be an unsigned integer in [0, 255]"
        self._repeat = value

    @builtins.property
    def color(self):
        """Message field 'color'."""
        return self._color

    @color.setter
    def color(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'color' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'color' field must be an unsigned integer in [0, 4294967295]"
        self._color = value
