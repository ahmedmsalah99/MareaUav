# generated from rosidl_generator_py/resource/_msg.py.em
# generated code does not contain a copyright notice

from copy import copy
import logging
import traceback


class Metaclass(type):
    """Metaclass of message 'AprilTagDetection'."""

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
            logger = logging.getLogger('rosidl_generator_py.AprilTagDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' + traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg_april_tag_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg_april_tag_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg_april_tag_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg_april_tag_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg_april_tag_detection
            from apriltag_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AprilTagDetection(metaclass=Metaclass):
    """Message class 'AprilTagDetection'."""

    __slots__ = [
        '_family',
        '_id',
        '_hamming',
        '_goodness',
        '_decision_margin',
        '_centre',
        '_corners',
        '_homography',
    ]

    _fields_and_field_types = {
        'family': 'string',
        'id': 'int32',
        'hamming': 'int32',
        'goodness': 'float32',
        'decision_margin': 'float32',
        'centre': 'apriltag_msgs/Point',
        'corners': 'apriltag_msgs/Point[4]',
        'homography': 'float64[9]',
    }

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.family = kwargs.get('family', str())
        self.id = kwargs.get('id', int())
        self.hamming = kwargs.get('hamming', int())
        self.goodness = kwargs.get('goodness', float())
        self.decision_margin = kwargs.get('decision_margin', float())
        from apriltag_msgs.msg import Point
        self.centre = kwargs.get('centre', Point())
        from apriltag_msgs.msg import Point
        self.corners = kwargs.get(
            'corners',
            [Point() for x in range(4)]
        )
        self.homography = kwargs.get(
            'homography',
            [float() for x in range(9)]
        )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = [s[1:] + '=' + repr(getattr(self, s, None)) for s in self.__slots__]
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.family != other.family:
            return False
        if self.id != other.id:
            return False
        if self.hamming != other.hamming:
            return False
        if self.goodness != other.goodness:
            return False
        if self.decision_margin != other.decision_margin:
            return False
        if self.centre != other.centre:
            return False
        if self.corners != other.corners:
            return False
        if self.homography != other.homography:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        return copy(cls._fields_and_field_types)

    @property
    def family(self):
        """Message field 'family'."""
        return self._family

    @family.setter
    def family(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'family' field must of type 'str'"
        self._family = value

    @property  # noqa: A003
    def id(self):
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @property
    def hamming(self):
        """Message field 'hamming'."""
        return self._hamming

    @hamming.setter
    def hamming(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hamming' field must of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'hamming' field must be an integer in [-2147483648, 2147483647]"
        self._hamming = value

    @property
    def goodness(self):
        """Message field 'goodness'."""
        return self._goodness

    @goodness.setter
    def goodness(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goodness' field must of type 'float'"
        self._goodness = value

    @property
    def decision_margin(self):
        """Message field 'decision_margin'."""
        return self._decision_margin

    @decision_margin.setter
    def decision_margin(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'decision_margin' field must of type 'float'"
        self._decision_margin = value

    @property
    def centre(self):
        """Message field 'centre'."""
        return self._centre

    @centre.setter
    def centre(self, value):
        if __debug__:
            from apriltag_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'centre' field must be a sub message of type 'Point'"
        self._centre = value

    @property
    def corners(self):
        """Message field 'corners'."""
        return self._corners

    @corners.setter
    def corners(self, value):
        if __debug__:
            from apriltag_msgs.msg import Point
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
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'corners' field must be a set or sequence with length 4 and each value of type 'Point'"
        self._corners = value

    @property
    def homography(self):
        """Message field 'homography'."""
        return self._homography

    @homography.setter
    def homography(self, value):
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
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'homography' field must be a set or sequence with length 9 and each value of type 'float'"
        self._homography = value
