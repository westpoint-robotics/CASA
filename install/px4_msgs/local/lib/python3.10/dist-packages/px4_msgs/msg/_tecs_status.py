# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/TecsStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TecsStatus(type):
    """Metaclass of message 'TecsStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TECS_MODE_NORMAL': 0,
        'TECS_MODE_UNDERSPEED': 1,
        'TECS_MODE_TAKEOFF': 2,
        'TECS_MODE_LAND': 3,
        'TECS_MODE_LAND_THROTTLELIM': 4,
        'TECS_MODE_BAD_DESCENT': 5,
        'TECS_MODE_CLIMBOUT': 6,
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
                'px4_msgs.msg.TecsStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tecs_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tecs_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tecs_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tecs_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tecs_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TECS_MODE_NORMAL': cls.__constants['TECS_MODE_NORMAL'],
            'TECS_MODE_UNDERSPEED': cls.__constants['TECS_MODE_UNDERSPEED'],
            'TECS_MODE_TAKEOFF': cls.__constants['TECS_MODE_TAKEOFF'],
            'TECS_MODE_LAND': cls.__constants['TECS_MODE_LAND'],
            'TECS_MODE_LAND_THROTTLELIM': cls.__constants['TECS_MODE_LAND_THROTTLELIM'],
            'TECS_MODE_BAD_DESCENT': cls.__constants['TECS_MODE_BAD_DESCENT'],
            'TECS_MODE_CLIMBOUT': cls.__constants['TECS_MODE_CLIMBOUT'],
        }

    @property
    def TECS_MODE_NORMAL(self):
        """Message constant 'TECS_MODE_NORMAL'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_NORMAL']

    @property
    def TECS_MODE_UNDERSPEED(self):
        """Message constant 'TECS_MODE_UNDERSPEED'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_UNDERSPEED']

    @property
    def TECS_MODE_TAKEOFF(self):
        """Message constant 'TECS_MODE_TAKEOFF'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_TAKEOFF']

    @property
    def TECS_MODE_LAND(self):
        """Message constant 'TECS_MODE_LAND'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_LAND']

    @property
    def TECS_MODE_LAND_THROTTLELIM(self):
        """Message constant 'TECS_MODE_LAND_THROTTLELIM'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_LAND_THROTTLELIM']

    @property
    def TECS_MODE_BAD_DESCENT(self):
        """Message constant 'TECS_MODE_BAD_DESCENT'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_BAD_DESCENT']

    @property
    def TECS_MODE_CLIMBOUT(self):
        """Message constant 'TECS_MODE_CLIMBOUT'."""
        return Metaclass_TecsStatus.__constants['TECS_MODE_CLIMBOUT']


class TecsStatus(metaclass=Metaclass_TecsStatus):
    """
    Message class 'TecsStatus'.

    Constants:
      TECS_MODE_NORMAL
      TECS_MODE_UNDERSPEED
      TECS_MODE_TAKEOFF
      TECS_MODE_LAND
      TECS_MODE_LAND_THROTTLELIM
      TECS_MODE_BAD_DESCENT
      TECS_MODE_CLIMBOUT
    """

    __slots__ = [
        '_timestamp',
        '_altitude_sp',
        '_altitude_filtered',
        '_height_rate_setpoint',
        '_height_rate',
        '_equivalent_airspeed_sp',
        '_true_airspeed_sp',
        '_true_airspeed_filtered',
        '_true_airspeed_derivative_sp',
        '_true_airspeed_derivative',
        '_true_airspeed_derivative_raw',
        '_true_airspeed_innovation',
        '_total_energy_error',
        '_energy_distribution_error',
        '_total_energy_rate_error',
        '_energy_distribution_rate_error',
        '_total_energy',
        '_total_energy_rate',
        '_total_energy_balance',
        '_total_energy_balance_rate',
        '_total_energy_sp',
        '_total_energy_rate_sp',
        '_total_energy_balance_sp',
        '_total_energy_balance_rate_sp',
        '_throttle_integ',
        '_pitch_integ',
        '_throttle_sp',
        '_pitch_sp_rad',
        '_throttle_trim',
        '_mode',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'altitude_sp': 'float',
        'altitude_filtered': 'float',
        'height_rate_setpoint': 'float',
        'height_rate': 'float',
        'equivalent_airspeed_sp': 'float',
        'true_airspeed_sp': 'float',
        'true_airspeed_filtered': 'float',
        'true_airspeed_derivative_sp': 'float',
        'true_airspeed_derivative': 'float',
        'true_airspeed_derivative_raw': 'float',
        'true_airspeed_innovation': 'float',
        'total_energy_error': 'float',
        'energy_distribution_error': 'float',
        'total_energy_rate_error': 'float',
        'energy_distribution_rate_error': 'float',
        'total_energy': 'float',
        'total_energy_rate': 'float',
        'total_energy_balance': 'float',
        'total_energy_balance_rate': 'float',
        'total_energy_sp': 'float',
        'total_energy_rate_sp': 'float',
        'total_energy_balance_sp': 'float',
        'total_energy_balance_rate_sp': 'float',
        'throttle_integ': 'float',
        'pitch_integ': 'float',
        'throttle_sp': 'float',
        'pitch_sp_rad': 'float',
        'throttle_trim': 'float',
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
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
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.altitude_sp = kwargs.get('altitude_sp', float())
        self.altitude_filtered = kwargs.get('altitude_filtered', float())
        self.height_rate_setpoint = kwargs.get('height_rate_setpoint', float())
        self.height_rate = kwargs.get('height_rate', float())
        self.equivalent_airspeed_sp = kwargs.get('equivalent_airspeed_sp', float())
        self.true_airspeed_sp = kwargs.get('true_airspeed_sp', float())
        self.true_airspeed_filtered = kwargs.get('true_airspeed_filtered', float())
        self.true_airspeed_derivative_sp = kwargs.get('true_airspeed_derivative_sp', float())
        self.true_airspeed_derivative = kwargs.get('true_airspeed_derivative', float())
        self.true_airspeed_derivative_raw = kwargs.get('true_airspeed_derivative_raw', float())
        self.true_airspeed_innovation = kwargs.get('true_airspeed_innovation', float())
        self.total_energy_error = kwargs.get('total_energy_error', float())
        self.energy_distribution_error = kwargs.get('energy_distribution_error', float())
        self.total_energy_rate_error = kwargs.get('total_energy_rate_error', float())
        self.energy_distribution_rate_error = kwargs.get('energy_distribution_rate_error', float())
        self.total_energy = kwargs.get('total_energy', float())
        self.total_energy_rate = kwargs.get('total_energy_rate', float())
        self.total_energy_balance = kwargs.get('total_energy_balance', float())
        self.total_energy_balance_rate = kwargs.get('total_energy_balance_rate', float())
        self.total_energy_sp = kwargs.get('total_energy_sp', float())
        self.total_energy_rate_sp = kwargs.get('total_energy_rate_sp', float())
        self.total_energy_balance_sp = kwargs.get('total_energy_balance_sp', float())
        self.total_energy_balance_rate_sp = kwargs.get('total_energy_balance_rate_sp', float())
        self.throttle_integ = kwargs.get('throttle_integ', float())
        self.pitch_integ = kwargs.get('pitch_integ', float())
        self.throttle_sp = kwargs.get('throttle_sp', float())
        self.pitch_sp_rad = kwargs.get('pitch_sp_rad', float())
        self.throttle_trim = kwargs.get('throttle_trim', float())
        self.mode = kwargs.get('mode', int())

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
        if self.altitude_sp != other.altitude_sp:
            return False
        if self.altitude_filtered != other.altitude_filtered:
            return False
        if self.height_rate_setpoint != other.height_rate_setpoint:
            return False
        if self.height_rate != other.height_rate:
            return False
        if self.equivalent_airspeed_sp != other.equivalent_airspeed_sp:
            return False
        if self.true_airspeed_sp != other.true_airspeed_sp:
            return False
        if self.true_airspeed_filtered != other.true_airspeed_filtered:
            return False
        if self.true_airspeed_derivative_sp != other.true_airspeed_derivative_sp:
            return False
        if self.true_airspeed_derivative != other.true_airspeed_derivative:
            return False
        if self.true_airspeed_derivative_raw != other.true_airspeed_derivative_raw:
            return False
        if self.true_airspeed_innovation != other.true_airspeed_innovation:
            return False
        if self.total_energy_error != other.total_energy_error:
            return False
        if self.energy_distribution_error != other.energy_distribution_error:
            return False
        if self.total_energy_rate_error != other.total_energy_rate_error:
            return False
        if self.energy_distribution_rate_error != other.energy_distribution_rate_error:
            return False
        if self.total_energy != other.total_energy:
            return False
        if self.total_energy_rate != other.total_energy_rate:
            return False
        if self.total_energy_balance != other.total_energy_balance:
            return False
        if self.total_energy_balance_rate != other.total_energy_balance_rate:
            return False
        if self.total_energy_sp != other.total_energy_sp:
            return False
        if self.total_energy_rate_sp != other.total_energy_rate_sp:
            return False
        if self.total_energy_balance_sp != other.total_energy_balance_sp:
            return False
        if self.total_energy_balance_rate_sp != other.total_energy_balance_rate_sp:
            return False
        if self.throttle_integ != other.throttle_integ:
            return False
        if self.pitch_integ != other.pitch_integ:
            return False
        if self.throttle_sp != other.throttle_sp:
            return False
        if self.pitch_sp_rad != other.pitch_sp_rad:
            return False
        if self.throttle_trim != other.throttle_trim:
            return False
        if self.mode != other.mode:
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
    def altitude_sp(self):
        """Message field 'altitude_sp'."""
        return self._altitude_sp

    @altitude_sp.setter
    def altitude_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'altitude_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._altitude_sp = value

    @builtins.property
    def altitude_filtered(self):
        """Message field 'altitude_filtered'."""
        return self._altitude_filtered

    @altitude_filtered.setter
    def altitude_filtered(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude_filtered' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'altitude_filtered' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._altitude_filtered = value

    @builtins.property
    def height_rate_setpoint(self):
        """Message field 'height_rate_setpoint'."""
        return self._height_rate_setpoint

    @height_rate_setpoint.setter
    def height_rate_setpoint(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height_rate_setpoint' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'height_rate_setpoint' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._height_rate_setpoint = value

    @builtins.property
    def height_rate(self):
        """Message field 'height_rate'."""
        return self._height_rate

    @height_rate.setter
    def height_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height_rate' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'height_rate' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._height_rate = value

    @builtins.property
    def equivalent_airspeed_sp(self):
        """Message field 'equivalent_airspeed_sp'."""
        return self._equivalent_airspeed_sp

    @equivalent_airspeed_sp.setter
    def equivalent_airspeed_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'equivalent_airspeed_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'equivalent_airspeed_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._equivalent_airspeed_sp = value

    @builtins.property
    def true_airspeed_sp(self):
        """Message field 'true_airspeed_sp'."""
        return self._true_airspeed_sp

    @true_airspeed_sp.setter
    def true_airspeed_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'true_airspeed_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'true_airspeed_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._true_airspeed_sp = value

    @builtins.property
    def true_airspeed_filtered(self):
        """Message field 'true_airspeed_filtered'."""
        return self._true_airspeed_filtered

    @true_airspeed_filtered.setter
    def true_airspeed_filtered(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'true_airspeed_filtered' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'true_airspeed_filtered' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._true_airspeed_filtered = value

    @builtins.property
    def true_airspeed_derivative_sp(self):
        """Message field 'true_airspeed_derivative_sp'."""
        return self._true_airspeed_derivative_sp

    @true_airspeed_derivative_sp.setter
    def true_airspeed_derivative_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'true_airspeed_derivative_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'true_airspeed_derivative_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._true_airspeed_derivative_sp = value

    @builtins.property
    def true_airspeed_derivative(self):
        """Message field 'true_airspeed_derivative'."""
        return self._true_airspeed_derivative

    @true_airspeed_derivative.setter
    def true_airspeed_derivative(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'true_airspeed_derivative' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'true_airspeed_derivative' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._true_airspeed_derivative = value

    @builtins.property
    def true_airspeed_derivative_raw(self):
        """Message field 'true_airspeed_derivative_raw'."""
        return self._true_airspeed_derivative_raw

    @true_airspeed_derivative_raw.setter
    def true_airspeed_derivative_raw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'true_airspeed_derivative_raw' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'true_airspeed_derivative_raw' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._true_airspeed_derivative_raw = value

    @builtins.property
    def true_airspeed_innovation(self):
        """Message field 'true_airspeed_innovation'."""
        return self._true_airspeed_innovation

    @true_airspeed_innovation.setter
    def true_airspeed_innovation(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'true_airspeed_innovation' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'true_airspeed_innovation' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._true_airspeed_innovation = value

    @builtins.property
    def total_energy_error(self):
        """Message field 'total_energy_error'."""
        return self._total_energy_error

    @total_energy_error.setter
    def total_energy_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_error' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_error' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_error = value

    @builtins.property
    def energy_distribution_error(self):
        """Message field 'energy_distribution_error'."""
        return self._energy_distribution_error

    @energy_distribution_error.setter
    def energy_distribution_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'energy_distribution_error' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'energy_distribution_error' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._energy_distribution_error = value

    @builtins.property
    def total_energy_rate_error(self):
        """Message field 'total_energy_rate_error'."""
        return self._total_energy_rate_error

    @total_energy_rate_error.setter
    def total_energy_rate_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_rate_error' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_rate_error' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_rate_error = value

    @builtins.property
    def energy_distribution_rate_error(self):
        """Message field 'energy_distribution_rate_error'."""
        return self._energy_distribution_rate_error

    @energy_distribution_rate_error.setter
    def energy_distribution_rate_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'energy_distribution_rate_error' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'energy_distribution_rate_error' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._energy_distribution_rate_error = value

    @builtins.property
    def total_energy(self):
        """Message field 'total_energy'."""
        return self._total_energy

    @total_energy.setter
    def total_energy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy = value

    @builtins.property
    def total_energy_rate(self):
        """Message field 'total_energy_rate'."""
        return self._total_energy_rate

    @total_energy_rate.setter
    def total_energy_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_rate' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_rate' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_rate = value

    @builtins.property
    def total_energy_balance(self):
        """Message field 'total_energy_balance'."""
        return self._total_energy_balance

    @total_energy_balance.setter
    def total_energy_balance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_balance' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_balance' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_balance = value

    @builtins.property
    def total_energy_balance_rate(self):
        """Message field 'total_energy_balance_rate'."""
        return self._total_energy_balance_rate

    @total_energy_balance_rate.setter
    def total_energy_balance_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_balance_rate' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_balance_rate' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_balance_rate = value

    @builtins.property
    def total_energy_sp(self):
        """Message field 'total_energy_sp'."""
        return self._total_energy_sp

    @total_energy_sp.setter
    def total_energy_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_sp = value

    @builtins.property
    def total_energy_rate_sp(self):
        """Message field 'total_energy_rate_sp'."""
        return self._total_energy_rate_sp

    @total_energy_rate_sp.setter
    def total_energy_rate_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_rate_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_rate_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_rate_sp = value

    @builtins.property
    def total_energy_balance_sp(self):
        """Message field 'total_energy_balance_sp'."""
        return self._total_energy_balance_sp

    @total_energy_balance_sp.setter
    def total_energy_balance_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_balance_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_balance_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_balance_sp = value

    @builtins.property
    def total_energy_balance_rate_sp(self):
        """Message field 'total_energy_balance_rate_sp'."""
        return self._total_energy_balance_rate_sp

    @total_energy_balance_rate_sp.setter
    def total_energy_balance_rate_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_energy_balance_rate_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'total_energy_balance_rate_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._total_energy_balance_rate_sp = value

    @builtins.property
    def throttle_integ(self):
        """Message field 'throttle_integ'."""
        return self._throttle_integ

    @throttle_integ.setter
    def throttle_integ(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle_integ' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'throttle_integ' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._throttle_integ = value

    @builtins.property
    def pitch_integ(self):
        """Message field 'pitch_integ'."""
        return self._pitch_integ

    @pitch_integ.setter
    def pitch_integ(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_integ' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'pitch_integ' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._pitch_integ = value

    @builtins.property
    def throttle_sp(self):
        """Message field 'throttle_sp'."""
        return self._throttle_sp

    @throttle_sp.setter
    def throttle_sp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle_sp' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'throttle_sp' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._throttle_sp = value

    @builtins.property
    def pitch_sp_rad(self):
        """Message field 'pitch_sp_rad'."""
        return self._pitch_sp_rad

    @pitch_sp_rad.setter
    def pitch_sp_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_sp_rad' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'pitch_sp_rad' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._pitch_sp_rad = value

    @builtins.property
    def throttle_trim(self):
        """Message field 'throttle_trim'."""
        return self._throttle_trim

    @throttle_trim.setter
    def throttle_trim(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle_trim' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'throttle_trim' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._throttle_trim = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value
