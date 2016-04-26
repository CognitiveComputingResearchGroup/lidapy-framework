class FrameworkMsg(object):
    def __init__(self):
        self._msg = None

    @property
    def serializable_msg(self):
        return self._msg

    @serializable_msg.setter
    def serializable_msg(self, value):
        self._msg = value


class Behavior(FrameworkMsg):
    def __init__(self):
        super(Behavior, self).__init__()
        super(Behavior, self).serializable_msg = Behavior.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Behavior as _Behavior
            return _Behavior
        except ImportError:
            return Behavior

    @property
    def id(self):
        pass

    @id.setter
    def id(self, value):
        pass

class Coalition(FrameworkMsg):
    def __init__(self):
        super(Coalition, self).__init__()
        super(Coalition, self).serializable_msg = Coalition.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Coalition as _Coalition
            return _Coalition
        except ImportError:
            return Coalition


class ConsciousContent(FrameworkMsg):
    def __init__(self):
        super(ConsciousContent, self).__init__()
        super(ConsciousContent, self).serializable_msg = ConsciousContent.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import ConsciousContent as _ConsciousContent
            return _ConsciousContent
        except ImportError:
            return ConsciousContent


class Cue(FrameworkMsg):
    def __init__(self):
        super(Cue, self).__init__()
        super(Cue, self).serializable_msg = Cue.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Cue as _Cue
            return _Cue
        except ImportError:
            return Cue

class Episode(FrameworkMsg):
    def __init__(self):
        super(Episode, self).__init__()
        super(Episode, self).serializable_msg = Episode.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Episode as _Episode
            return _Episode
        except ImportError:
            return Episode


class Feature(FrameworkMsg):
    def __init__(self):
        super(Feature, self).__init__()
        super(Feature, self).serializable_msg = Feature.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Feature as _Feature
            return _Feature
        except ImportError:
            return Feature


class Percept(FrameworkMsg):
    def __init__(self):
        super(Percept, self).__init__()
        super(Percept, self).serializable_msg = Percept.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Percept as _Percept
            return _Percept
        except ImportError:
            return Percept


class SpatialMap(FrameworkMsg):
    def __init__(self):
        super(SpatialMap, self).__init__()
        super(SpatialMap, self).serializable_msg = SpatialMap.msg_type()()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import SpatialMap as _SpatialMap
            return _SpatialMap
        except ImportError:
            return SpatialMap

