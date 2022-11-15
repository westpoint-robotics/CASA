# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/WheelEncoders.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WheelEncoders(type):
    """Metaclass of message 'WheelEncoders'."""

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
            module = import_type_support('px4_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'px4_msgs.msg.WheelEncoders')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__wheel_encoders
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__wheel_encoders
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__wheel_encoders
            cls._TYPE_SUPPORT = module.type_support_msg__msg__wheel_encoders
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__wheel_encoders

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WheelEncoders(metaclass=Metaclass_WheelEncoders):
    """Message class 'WheelEncoders'."""

    __slots__ = [
        '_timestamp',
        '_encoder_position',
        '_speed',
        '_pulses_per_rev',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'encoder_position': 'int64',
        'speed': 'int32',
        'pulses_per_rev': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.encoder_position = kwargs.get('encoder_position', int())
        self.speed = kwargs.get('speed', int())
        self.pulses_per_rev = kwargs.get('pulses_per_rev', int())

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
        if self.encoder_position != other.encoder_position:
            return False
        if self.speed != other.speed:
            return False
        if self.pulses_per_rev != other.pulses_per_rev:
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
    def encoder_position(self):
        """Message field 'encoder_position'."""
        return self._encoder_position

    @encoder_position.setter
    def encoder_position(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'encoder_position' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'encoder_position' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._encoder_position = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'speed' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'speed' field must be an integer in [-2147483648, 2147483647]"
        self._speed = value

    @builtins.property
    def pulses_per_rev(self):
        """Message field 'pulses_per_rev'."""
        return self._pulses_per_rev

    @pulses_per_rev.setter
    def pulses_per_rev(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pulses_per_rev' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'pulses_per_rev' field must be an unsigned integer in [0, 4294967295]"
        self._pulses_per_rev = value
