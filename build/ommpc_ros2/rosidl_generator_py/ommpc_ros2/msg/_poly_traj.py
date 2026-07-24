# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ommpc_ros2:msg/PolyTraj.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'coef_x'
# Member 'coef_y'
# Member 'coef_z'
# Member 'duration'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PolyTraj(type):
    """Metaclass of message 'PolyTraj'."""

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
                'ommpc_ros2.msg.PolyTraj')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__poly_traj
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__poly_traj
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__poly_traj
            cls._TYPE_SUPPORT = module.type_support_msg__msg__poly_traj
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__poly_traj

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PolyTraj(metaclass=Metaclass_PolyTraj):
    """Message class 'PolyTraj'."""

    __slots__ = [
        '_drone_id',
        '_traj_id',
        '_start_time',
        '_order',
        '_coef_x',
        '_coef_y',
        '_coef_z',
        '_duration',
    ]

    _fields_and_field_types = {
        'drone_id': 'int16',
        'traj_id': 'int32',
        'start_time': 'builtin_interfaces/Time',
        'order': 'uint8',
        'coef_x': 'sequence<float>',
        'coef_y': 'sequence<float>',
        'coef_z': 'sequence<float>',
        'duration': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.drone_id = kwargs.get('drone_id', int())
        self.traj_id = kwargs.get('traj_id', int())
        from builtin_interfaces.msg import Time
        self.start_time = kwargs.get('start_time', Time())
        self.order = kwargs.get('order', int())
        self.coef_x = array.array('f', kwargs.get('coef_x', []))
        self.coef_y = array.array('f', kwargs.get('coef_y', []))
        self.coef_z = array.array('f', kwargs.get('coef_z', []))
        self.duration = array.array('f', kwargs.get('duration', []))

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
        if self.drone_id != other.drone_id:
            return False
        if self.traj_id != other.traj_id:
            return False
        if self.start_time != other.start_time:
            return False
        if self.order != other.order:
            return False
        if self.coef_x != other.coef_x:
            return False
        if self.coef_y != other.coef_y:
            return False
        if self.coef_z != other.coef_z:
            return False
        if self.duration != other.duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def drone_id(self):
        """Message field 'drone_id'."""
        return self._drone_id

    @drone_id.setter
    def drone_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'drone_id' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'drone_id' field must be an integer in [-32768, 32767]"
        self._drone_id = value

    @builtins.property
    def traj_id(self):
        """Message field 'traj_id'."""
        return self._traj_id

    @traj_id.setter
    def traj_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'traj_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'traj_id' field must be an integer in [-2147483648, 2147483647]"
        self._traj_id = value

    @builtins.property
    def start_time(self):
        """Message field 'start_time'."""
        return self._start_time

    @start_time.setter
    def start_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'start_time' field must be a sub message of type 'Time'"
        self._start_time = value

    @builtins.property
    def order(self):
        """Message field 'order'."""
        return self._order

    @order.setter
    def order(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'order' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'order' field must be an unsigned integer in [0, 255]"
        self._order = value

    @builtins.property
    def coef_x(self):
        """Message field 'coef_x'."""
        return self._coef_x

    @coef_x.setter
    def coef_x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'coef_x' array.array() must have the type code of 'f'"
            self._coef_x = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'coef_x' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._coef_x = array.array('f', value)

    @builtins.property
    def coef_y(self):
        """Message field 'coef_y'."""
        return self._coef_y

    @coef_y.setter
    def coef_y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'coef_y' array.array() must have the type code of 'f'"
            self._coef_y = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'coef_y' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._coef_y = array.array('f', value)

    @builtins.property
    def coef_z(self):
        """Message field 'coef_z'."""
        return self._coef_z

    @coef_z.setter
    def coef_z(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'coef_z' array.array() must have the type code of 'f'"
            self._coef_z = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'coef_z' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._coef_z = array.array('f', value)

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'duration' array.array() must have the type code of 'f'"
            self._duration = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'duration' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._duration = array.array('f', value)
