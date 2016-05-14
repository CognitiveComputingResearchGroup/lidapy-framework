from lida.msg import Behaviors, Coalitions, ConsciousContent, Cues, Episodes, Features, Percepts, SpatialMaps

from lidapy.util import comm, logger


class FrameworkTopic(object):
    def __init__(self, topic_name, msg_type, queue_size=0):
        super(FrameworkTopic, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
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
