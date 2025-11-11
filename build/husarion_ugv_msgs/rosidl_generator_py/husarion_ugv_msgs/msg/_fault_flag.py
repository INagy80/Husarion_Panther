# generated from rosidl_generator_py/resource/_idl.py.em
# with input from husarion_ugv_msgs:msg/FaultFlag.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FaultFlag(type):
    """Metaclass of message 'FaultFlag'."""

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
                'husarion_ugv_msgs.msg.FaultFlag')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__fault_flag
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__fault_flag
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__fault_flag
            cls._TYPE_SUPPORT = module.type_support_msg__msg__fault_flag
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__fault_flag

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FaultFlag(metaclass=Metaclass_FaultFlag):
    """Message class 'FaultFlag'."""

    __slots__ = [
        '_overheat',
        '_overvoltage',
        '_undervoltage',
        '_short_circuit',
        '_emergency_stop',
        '_motor_or_sensor_setup_fault',
        '_mosfet_failure',
        '_default_config_loaded_at_startup',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'overheat': 'boolean',
        'overvoltage': 'boolean',
        'undervoltage': 'boolean',
        'short_circuit': 'boolean',
        'emergency_stop': 'boolean',
        'motor_or_sensor_setup_fault': 'boolean',
        'mosfet_failure': 'boolean',
        'default_config_loaded_at_startup': 'boolean',
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
        self.overheat = kwargs.get('overheat', bool())
        self.overvoltage = kwargs.get('overvoltage', bool())
        self.undervoltage = kwargs.get('undervoltage', bool())
        self.short_circuit = kwargs.get('short_circuit', bool())
        self.emergency_stop = kwargs.get('emergency_stop', bool())
        self.motor_or_sensor_setup_fault = kwargs.get('motor_or_sensor_setup_fault', bool())
        self.mosfet_failure = kwargs.get('mosfet_failure', bool())
        self.default_config_loaded_at_startup = kwargs.get('default_config_loaded_at_startup', bool())

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
        if self.overheat != other.overheat:
            return False
        if self.overvoltage != other.overvoltage:
            return False
        if self.undervoltage != other.undervoltage:
            return False
        if self.short_circuit != other.short_circuit:
            return False
        if self.emergency_stop != other.emergency_stop:
            return False
        if self.motor_or_sensor_setup_fault != other.motor_or_sensor_setup_fault:
            return False
        if self.mosfet_failure != other.mosfet_failure:
            return False
        if self.default_config_loaded_at_startup != other.default_config_loaded_at_startup:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def overheat(self):
        """Message field 'overheat'."""
        return self._overheat

    @overheat.setter
    def overheat(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'overheat' field must be of type 'bool'"
        self._overheat = value

    @builtins.property
    def overvoltage(self):
        """Message field 'overvoltage'."""
        return self._overvoltage

    @overvoltage.setter
    def overvoltage(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'overvoltage' field must be of type 'bool'"
        self._overvoltage = value

    @builtins.property
    def undervoltage(self):
        """Message field 'undervoltage'."""
        return self._undervoltage

    @undervoltage.setter
    def undervoltage(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'undervoltage' field must be of type 'bool'"
        self._undervoltage = value

    @builtins.property
    def short_circuit(self):
        """Message field 'short_circuit'."""
        return self._short_circuit

    @short_circuit.setter
    def short_circuit(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'short_circuit' field must be of type 'bool'"
        self._short_circuit = value

    @builtins.property
    def emergency_stop(self):
        """Message field 'emergency_stop'."""
        return self._emergency_stop

    @emergency_stop.setter
    def emergency_stop(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'emergency_stop' field must be of type 'bool'"
        self._emergency_stop = value

    @builtins.property
    def motor_or_sensor_setup_fault(self):
        """Message field 'motor_or_sensor_setup_fault'."""
        return self._motor_or_sensor_setup_fault

    @motor_or_sensor_setup_fault.setter
    def motor_or_sensor_setup_fault(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'motor_or_sensor_setup_fault' field must be of type 'bool'"
        self._motor_or_sensor_setup_fault = value

    @builtins.property
    def mosfet_failure(self):
        """Message field 'mosfet_failure'."""
        return self._mosfet_failure

    @mosfet_failure.setter
    def mosfet_failure(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'mosfet_failure' field must be of type 'bool'"
        self._mosfet_failure = value

    @builtins.property
    def default_config_loaded_at_startup(self):
        """Message field 'default_config_loaded_at_startup'."""
        return self._default_config_loaded_at_startup

    @default_config_loaded_at_startup.setter
    def default_config_loaded_at_startup(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'default_config_loaded_at_startup' field must be of type 'bool'"
        self._default_config_loaded_at_startup = value
