# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ommpc_ros2:msg/ReferenceState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReferenceState(type):
    """Metaclass of message 'ReferenceState'."""

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
            module = import_type_support('ommpc_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ommpc_ros2.msg.ReferenceState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__reference_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__reference_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__reference_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__reference_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__reference_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReferenceState(metaclass=Metaclass_ReferenceState):
    """Message class 'ReferenceState'."""

    __slots__ = [
        '_timestamp',
        '_position',
        '_velocity',
        '_quaternion',
        '_body_rates',
        '_thrust',
    ]

    _fields_and_field_types = {
        'timestamp': 'builtin_interfaces/Time',
        'position': 'geometry_msgs/Point',
        'velocity': 'geometry_msgs/Vector3',
        'quaternion': 'geometry_msgs/Quaternion',
        'body_rates': 'geometry_msgs/Vector3',
        'thrust': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.timestamp = kwargs.get('timestamp', Time())
        from geometry_msgs.msg import Point
        self.position = kwargs.get('position', Point())
        from geometry_msgs.msg import Vector3
        self.velocity = kwargs.get('velocity', Vector3())
        from geometry_msgs.msg import Quaternion
        self.quaternion = kwargs.get('quaternion', Quaternion())
        from geometry_msgs.msg import Vector3
        self.body_rates = kwargs.get('body_rates', Vector3())
        self.thrust = kwargs.get('thrust', float())

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
        if self.position != other.position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.quaternion != other.quaternion:
            return False
        if self.body_rates != other.body_rates:
            return False
        if self.thrust != other.thrust:
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
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp' field must be a sub message of type 'Time'"
        self._timestamp = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position' field must be a sub message of type 'Point'"
        self._position = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'velocity' field must be a sub message of type 'Vector3'"
        self._velocity = value

    @builtins.property
    def quaternion(self):
        """Message field 'quaternion'."""
        return self._quaternion

    @quaternion.setter
    def quaternion(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'quaternion' field must be a sub message of type 'Quaternion'"
        self._quaternion = value

    @builtins.property
    def body_rates(self):
        """Message field 'body_rates'."""
        return self._body_rates

    @body_rates.setter
    def body_rates(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'body_rates' field must be a sub message of type 'Vector3'"
        self._body_rates = value

    @builtins.property
    def thrust(self):
        """Message field 'thrust'."""
        return self._thrust

    @thrust.setter
    def thrust(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust = value
