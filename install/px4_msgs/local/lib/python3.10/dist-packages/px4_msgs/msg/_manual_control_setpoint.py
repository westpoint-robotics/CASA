# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/ManualControlSetpoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ManualControlSetpoint(type):
    """Metaclass of message 'ManualControlSetpoint'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SOURCE_UNKNOWN': 0,
        'SOURCE_RC': 1,
        'SOURCE_MAVLINK_0': 2,
        'SOURCE_MAVLINK_1': 3,
        'SOURCE_MAVLINK_2': 4,
        'SOURCE_MAVLINK_3': 5,
        'SOURCE_MAVLINK_4': 6,
        'SOURCE_MAVLINK_5': 7,
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
                'px4_msgs.msg.ManualControlSetpoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__manual_control_setpoint
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__manual_control_setpoint
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__manual_control_setpoint
            cls._TYPE_SUPPORT = module.type_support_msg__msg__manual_control_setpoint
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__manual_control_setpoint

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SOURCE_UNKNOWN': cls.__constants['SOURCE_UNKNOWN'],
            'SOURCE_RC': cls.__constants['SOURCE_RC'],
            'SOURCE_MAVLINK_0': cls.__constants['SOURCE_MAVLINK_0'],
            'SOURCE_MAVLINK_1': cls.__constants['SOURCE_MAVLINK_1'],
            'SOURCE_MAVLINK_2': cls.__constants['SOURCE_MAVLINK_2'],
            'SOURCE_MAVLINK_3': cls.__constants['SOURCE_MAVLINK_3'],
            'SOURCE_MAVLINK_4': cls.__constants['SOURCE_MAVLINK_4'],
            'SOURCE_MAVLINK_5': cls.__constants['SOURCE_MAVLINK_5'],
        }

    @property
    def SOURCE_UNKNOWN(self):
        """Message constant 'SOURCE_UNKNOWN'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_UNKNOWN']

    @property
    def SOURCE_RC(self):
        """Message constant 'SOURCE_RC'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_RC']

    @property
    def SOURCE_MAVLINK_0(self):
        """Message constant 'SOURCE_MAVLINK_0'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_MAVLINK_0']

    @property
    def SOURCE_MAVLINK_1(self):
        """Message constant 'SOURCE_MAVLINK_1'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_MAVLINK_1']

    @property
    def SOURCE_MAVLINK_2(self):
        """Message constant 'SOURCE_MAVLINK_2'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_MAVLINK_2']

    @property
    def SOURCE_MAVLINK_3(self):
        """Message constant 'SOURCE_MAVLINK_3'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_MAVLINK_3']

    @property
    def SOURCE_MAVLINK_4(self):
        """Message constant 'SOURCE_MAVLINK_4'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_MAVLINK_4']

    @property
    def SOURCE_MAVLINK_5(self):
        """Message constant 'SOURCE_MAVLINK_5'."""
        return Metaclass_ManualControlSetpoint.__constants['SOURCE_MAVLINK_5']


class ManualControlSetpoint(metaclass=Metaclass_ManualControlSetpoint):
    """
    Message class 'ManualControlSetpoint'.

    Constants:
      SOURCE_UNKNOWN
      SOURCE_RC
      SOURCE_MAVLINK_0
      SOURCE_MAVLINK_1
      SOURCE_MAVLINK_2
      SOURCE_MAVLINK_3
      SOURCE_MAVLINK_4
      SOURCE_MAVLINK_5
    """

    __slots__ = [
        '_timestamp',
        '_timestamp_sample',
        '_valid',
        '_data_source',
        '_x',
        '_y',
        '_z',
        '_r',
        '_flaps',
        '_aux1',
        '_aux2',
        '_aux3',
        '_aux4',
        '_aux5',
        '_aux6',
        '_sticks_moving',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'timestamp_sample': 'uint64',
        'valid': 'boolean',
        'data_source': 'uint8',
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'r': 'float',
        'flaps': 'float',
        'aux1': 'float',
        'aux2': 'float',
        'aux3': 'float',
        'aux4': 'float',
        'aux5': 'float',
        'aux6': 'float',
        'sticks_moving': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.timestamp_sample = kwargs.get('timestamp_sample', int())
        self.valid = kwargs.get('valid', bool())
        self.data_source = kwargs.get('data_source', int())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.r = kwargs.get('r', float())
        self.flaps = kwargs.get('flaps', float())
        self.aux1 = kwargs.get('aux1', float())
        self.aux2 = kwargs.get('aux2', float())
        self.aux3 = kwargs.get('aux3', float())
        self.aux4 = kwargs.get('aux4', float())
        self.aux5 = kwargs.get('aux5', float())
        self.aux6 = kwargs.get('aux6', float())
        self.sticks_moving = kwargs.get('sticks_moving', bool())

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
        if self.timestamp_sample != other.timestamp_sample:
            return False
        if self.valid != other.valid:
            return False
        if self.data_source != other.data_source:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.r != other.r:
            return False
        if self.flaps != other.flaps:
            return False
        if self.aux1 != other.aux1:
            return False
        if self.aux2 != other.aux2:
            return False
        if self.aux3 != other.aux3:
            return False
        if self.aux4 != other.aux4:
            return False
        if self.aux5 != other.aux5:
            return False
        if self.aux6 != other.aux6:
            return False
        if self.sticks_moving != other.sticks_moving:
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
    def timestamp_sample(self):
        """Message field 'timestamp_sample'."""
        return self._timestamp_sample

    @timestamp_sample.setter
    def timestamp_sample(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp_sample' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp_sample' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp_sample = value

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value

    @builtins.property
    def data_source(self):
        """Message field 'data_source'."""
        return self._data_source

    @data_source.setter
    def data_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'data_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'data_source' field must be an unsigned integer in [0, 255]"
        self._data_source = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'x' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'y' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'z' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._z = value

    @builtins.property
    def r(self):
        """Message field 'r'."""
        return self._r

    @r.setter
    def r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'r' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._r = value

    @builtins.property
    def flaps(self):
        """Message field 'flaps'."""
        return self._flaps

    @flaps.setter
    def flaps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'flaps' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'flaps' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._flaps = value

    @builtins.property
    def aux1(self):
        """Message field 'aux1'."""
        return self._aux1

    @aux1.setter
    def aux1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux1' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'aux1' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._aux1 = value

    @builtins.property
    def aux2(self):
        """Message field 'aux2'."""
        return self._aux2

    @aux2.setter
    def aux2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux2' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'aux2' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._aux2 = value

    @builtins.property
    def aux3(self):
        """Message field 'aux3'."""
        return self._aux3

    @aux3.setter
    def aux3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux3' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'aux3' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._aux3 = value

    @builtins.property
    def aux4(self):
        """Message field 'aux4'."""
        return self._aux4

    @aux4.setter
    def aux4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux4' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'aux4' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._aux4 = value

    @builtins.property
    def aux5(self):
        """Message field 'aux5'."""
        return self._aux5

    @aux5.setter
    def aux5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux5' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'aux5' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._aux5 = value

    @builtins.property
    def aux6(self):
        """Message field 'aux6'."""
        return self._aux6

    @aux6.setter
    def aux6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux6' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'aux6' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._aux6 = value

    @builtins.property
    def sticks_moving(self):
        """Message field 'sticks_moving'."""
        return self._sticks_moving

    @sticks_moving.setter
    def sticks_moving(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sticks_moving' field must be of type 'bool'"
        self._sticks_moving = value
