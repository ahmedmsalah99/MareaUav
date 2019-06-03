# generated from rosidl_generator_py/resource/_msg.py.em
# generated code does not contain a copyright notice

from copy import copy
import logging
import traceback


class Metaclass(type):
    """Metaclass of message 'AprilTagDetectionArray'."""

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
            module = import_type_support('apriltag_msgs')
        except ImportError:
            logger = logging.getLogger('rosidl_generator_py.AprilTagDetectionArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' + traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg_april_tag_detection_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg_april_tag_detection_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg_april_tag_detection_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg_april_tag_detection_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg_april_tag_detection_array
            from apriltag_msgs.msg import AprilTagDetection
            if AprilTagDetection.__class__._TYPE_SUPPORT is None:
                AprilTagDetection.__class__.__import_type_support__()
            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AprilTagDetectionArray(metaclass=Metaclass):
    """Message class 'AprilTagDetectionArray'."""

    __slots__ = [
        '_header',
        '_detections',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'detections': 'apriltag_msgs/AprilTagDetection[]',
    }

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.detections = kwargs.get('detections', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = [s[1:] + '=' + repr(getattr(self, s, None)) for s in self.__slots__]
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.detections != other.detections:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def detections(self):
        """Message field 'detections'."""
        return self._detections

    @detections.setter
    def detections(self, value):
        if __debug__:
            from apriltag_msgs.msg import AprilTagDetection
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
                 all(isinstance(v, AprilTagDetection) for v in value) and
                 True), \
                "The 'detections' field must be a set or sequence and each value of type 'AprilTagDetection'"
        self._detections = value
