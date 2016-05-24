from lida.msg import CognitiveContent

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
    "selected_behaviors": FrameworkTopic("selected_behaviors", CognitiveContent),
    "candidate_behaviors": FrameworkTopic("candidate_behaviors", CognitiveContent),
    "global_broadcast": FrameworkTopic("global_broadcast", CognitiveContent),
    "episodes": FrameworkTopic("episodes", CognitiveContent),
    "workspace_cues": FrameworkTopic("workspace_cues", CognitiveContent),
    "workspace_coalitions": FrameworkTopic("workspace_coalitions", CognitiveContent),
    "detected_features": FrameworkTopic("detected_features", CognitiveContent),
    "percepts": FrameworkTopic("percepts", CognitiveContent),
    "spatial_maps": FrameworkTopic("spatial_maps", CognitiveContent),
    "dorsal_stream": FrameworkTopic("dorsal_stream", CognitiveContent),
    "ventral_stream": FrameworkTopic("ventral_stream", CognitiveContent),
}
