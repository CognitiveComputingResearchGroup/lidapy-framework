from lidapy.util import comm, logger


class FrameworkTopic(object):
    def __init__(self, topic_name, msg_class, queue_size=0):
        super(FrameworkTopic, self).__init__()

        self.topic_name = topic_name
        self.msg_class = msg_class
        self.msg_type = msg_class.msg_type()
        self.queue_size = queue_size

    def register_subscriber(self, callback=None, callback_args=None):
        sub_args = {"topic": self.topic_name}

        if callback_args is not None:
            sub_args.update(callback_args)

        comm.register_subscriber(self.topic_name, self.msg_type, callback, sub_args)

    def get_publisher(self):
        return FrameworkTopicPublisher(self.topic_name, self.msg_type, self.queue_size)


class FrameworkTopicPublisher(object):

    def __init__(self, topic_name, msg_type, queue_size=0):
        super(FrameworkTopicPublisher, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size

        self._publisher \
            = comm.get_publisher(self.topic_name,
                                 self.msg_type,
                                 queue_size=self.queue_size)

    def publish(self, msg):
        logger.debug("Publishing msg to topic [{}]".format(self.topic_name))

        comm.publish_message(self._publisher, msg)


class FrameworkMsg(object):
    def __init__(self):
        super(FrameworkMsg, self).__init__()

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


class Behaviors(FrameworkMsg):
    def __init__(self):
        super(Behaviors, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Behaviors as _Behaviors
            return _Behaviors
        except ImportError:
            return Behaviors

    def _create_serializable_msg(self):
        return Behaviors.msg_type()()


class Coalitions(FrameworkMsg):
    def __init__(self):
        super(Coalitions, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Coalitions as _Coalitions
            return _Coalitions
        except ImportError:
            return Coalitions

    def _create_serializable_msg(self):
        return Coalitions.msg_type()()


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


class Cues(FrameworkMsg):
    def __init__(self):
        super(Cues, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Cues as _Cues
            return _Cues
        except ImportError:
            return Cues

    def _create_serializable_msg(self):
        return Episodes.msg_type()()


class Episodes(FrameworkMsg):
    def __init__(self):
        super(Episodes, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Episodes as _Episodes
            return _Episodes
        except ImportError:
            return Episodes

    def _create_serializable_msg(self):
        return Episodes.msg_type()()


class Features(FrameworkMsg):
    def __init__(self):
        super(Features, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Features as _Features
            return _Features
        except ImportError:
            return Features

    def _create_serializable_msg(self):
        return Features.msg_type()()


class Percepts(FrameworkMsg):
    def __init__(self):
        super(Percepts, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import Percepts as _Percepts
            return _Percepts
        except ImportError:
            return Percepts

    def _create_serializable_msg(self):
        return Percepts.msg_type()()


class SpatialMaps(FrameworkMsg):
    def __init__(self):
        super(SpatialMaps, self).__init__()

    @staticmethod
    def msg_type():
        try:
            # ROS specific imports
            from lida.msg import SpatialMaps as _SpatialMaps
            return _SpatialMaps
        except ImportError:
            return SpatialMaps

    def _create_serializable_msg(self):
        return SpatialMaps.msg_type()()


built_in_topics = {
    "/lida/selected_behaviors": FrameworkTopic("/lida/selected_behaviors", Behaviors),
    "/lida/candidate_behaviors": FrameworkTopic("/lida/candidate_behaviors", Behaviors),
    "/lida/global_broadcast": FrameworkTopic("/lida/global_broadcast", ConsciousContent),
    "/lida/episodes": FrameworkTopic("/lida/episodes", Episodes),
    "/lida/workspace_cues": FrameworkTopic("/lida/workspace_cues", Cues),
    "/lida/workspace_coalitions": FrameworkTopic("/lida/workspace_coalitions", Coalitions),
    "/lida/detected_features": FrameworkTopic("/lida/detected_features", Features),
    "/lida/percepts": FrameworkTopic("/lida/percepts", Percepts),
    "/lida/spatial_maps": FrameworkTopic("/lida/spatial_maps", SpatialMaps),
    "/lida/dorsal_stream": FrameworkTopic("/lida/dorsal_stream", Features),
    "/lida/ventral_stream": FrameworkTopic("/lida/ventral_stream", Features),
}
