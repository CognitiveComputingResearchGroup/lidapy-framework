class FrameworkMsg(object):
    def __init__(self):
        self._msg = self._create_serializable_msg()

    def _create_serializable_msg(self):
        pass

    def serialize(self):
        return self._msg

    @property
    def id(self):
        return self._msg.id

    @id.setter
    def id(self, value):
        self._msg.id = value


class Behavior(FrameworkMsg):
    def __init__(self):
        super(Behavior, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Behavior as _Behavior
            return _Behavior
        except ImportError:
            return Behavior

    def _create_serializable_msg(self):
        return Behavior.msg_type()()


class Coalition(FrameworkMsg):
    def __init__(self):
        super(Coalition, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Coalition as _Coalition
            return _Coalition
        except ImportError:
            return Coalition

    def _create_serializable_msg(self):
        return Coalition.msg_type()()


class ConsciousContent(FrameworkMsg):
    def __init__(self):
        super(ConsciousContent, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import ConsciousContent as _ConsciousContent
            return _ConsciousContent
        except ImportError:
            return ConsciousContent

    def _create_serializable_msg(self):
        return ConsciousContent.msg_type()()


class Cue(FrameworkMsg):
    def __init__(self):
        super(Cue, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Cue as _Cue
            return _Cue
        except ImportError:
            return Cue

    def _create_serializable_msg(self):
        return Episode.msg_type()()


class Episode(FrameworkMsg):
    def __init__(self):
        super(Episode, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Episode as _Episode
            return _Episode
        except ImportError:
            return Episode

    def _create_serializable_msg(self):
        return Episode.msg_type()()


class Feature(FrameworkMsg):
    def __init__(self):
        super(Feature, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Feature as _Feature
            return _Feature
        except ImportError:
            return Feature

    def _create_serializable_msg(self):
        return Feature.msg_type()()


class Percept(FrameworkMsg):
    def __init__(self):
        super(Percept, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Percept as _Percept
            return _Percept
        except ImportError:
            return Percept

    def _create_serializable_msg(self):
        return Percept.msg_type()()


class SpatialMap(FrameworkMsg):
    def __init__(self):
        super(SpatialMap, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import SpatialMap as _SpatialMap
            return _SpatialMap
        except ImportError:
            return SpatialMap

    def _create_serializable_msg(self):
        return SpatialMap.msg_type()()
