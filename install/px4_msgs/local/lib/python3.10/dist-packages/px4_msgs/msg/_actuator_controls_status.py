# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/ActuatorControlsStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'control_power'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ActuatorControlsStatus(type):
    """Metaclass of message 'ActuatorControlsStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'INDEX_ROLL': 0,
        'INDEX_PITCH': 1,
        'INDEX_YAW': 2,
        'INDEX_THROTTLE': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('px4_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'px4_msgs.msg.ActuatorControlsStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__actuator_controls_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__actuator_controls_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__actuator_controls_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__actuator_controls_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__actuator_controls_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'INDEX_ROLL': cls.__constants['INDEX_ROLL'],
            'INDEX_PITCH': cls.__constants['INDEX_PITCH'],
            'INDEX_YAW': cls.__constants['INDEX_YAW'],
            'INDEX_THROTTLE': cls.__constants['INDEX_THROTTLE'],
        }

    @property
    def INDEX_ROLL(self):
        """Message constant 'INDEX_ROLL'."""
        return Metaclass_ActuatorControlsStatus.__constants['INDEX_ROLL']

    @property
    def INDEX_PITCH(self):
        """Message constant 'INDEX_PITCH'."""
        return Metaclass_ActuatorControlsStatus.__constants['INDEX_PITCH']

    @property
    def INDEX_YAW(self):
        """Message constant 'INDEX_YAW'."""
        return Metaclass_ActuatorControlsStatus.__constants['INDEX_YAW']

    @property
    def INDEX_THROTTLE(self):
        """Message constant 'INDEX_THROTTLE'."""
        return Metaclass_ActuatorControlsStatus.__constants['INDEX_THROTTLE']


class ActuatorControlsStatus(metaclass=Metaclass_ActuatorControlsStatus):
    """
    Message class 'ActuatorControlsStatus'.

    Constants:
      INDEX_ROLL
      INDEX_PITCH
      INDEX_YAW
      INDEX_THROTTLE
    """

    __slots__ = [
        '_timestamp',
        '_control_power',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'control_power': 'float[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        if 'control_power' not in kwargs:
            self.control_power = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.control_power = numpy.array(kwargs.get('control_power'), dtype=numpy.float32)
            assert self.control_power.shape == (4, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.timestamp != other.timestamp:
            return False
        if all(self.control_power != other.control_power):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp = value

    @builtins.property
    def control_power(self):
        """Message field 'control_power'."""
        return self._control_power

    @control_power.setter
    def control_power(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'control_power' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'control_power' numpy.ndarray() must have a size of 4"
            self._control_power = value
            return
        if __debug__:
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
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(val >= -3.402823e+38 and val <= 3.402823e+38 for val in value)), \
                "The 'control_power' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282299999999994960115009090224128000.000000, 340282299999999994960115009090224128000.000000]"
        self._control_power = numpy.array(value, dtype=numpy.float32)
