# generated from rosidl_generator_py/resource/_msg.py.em
# generated code does not contain a copyright notice

from copy import copy
import logging
import traceback


class Metaclass(type):
    """Metaclass of message 'UltrasonicMsg'."""

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
            module = import_type_support('ultrasonic_sensor')
        except ImportError:
            logger = logging.getLogger('rosidl_generator_py.UltrasonicMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' + traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg_ultrasonic_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg_ultrasonic_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg_ultrasonic_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg_ultrasonic_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg_ultrasonic_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UltrasonicMsg(metaclass=Metaclass):
    """Message class 'UltrasonicMsg'."""

    __slots__ = [
        '_z_position',
        '_z_velocity',
    ]

    _fields_and_field_types = {
        'z_position': 'float64',
        'z_velocity': 'float64',
    }

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.z_position = kwargs.get('z_position', float())
        self.z_velocity = kwargs.get('z_velocity', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = [s[1:] + '=' + repr(getattr(self, s, None)) for s in self.__slots__]
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.z_position != other.z_position:
            return False
        if self.z_velocity != other.z_velocity:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        return copy(cls._fields_and_field_types)

    @property
    def z_position(self):
        """Message field 'z_position'."""
        return self._z_position

    @z_position.setter
    def z_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_position' field must of type 'float'"
        self._z_position = value

    @property
    def z_velocity(self):
        """Message field 'z_velocity'."""
        return self._z_velocity

    @z_velocity.setter
    def z_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_velocity' field must of type 'float'"
        self._z_velocity = value
