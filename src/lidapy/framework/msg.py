from cPickle import dumps, loads  # Object Serialization / Deserialization

from std_msgs.msg import String

from lidapy.util import comm, logger

class FrameworkTopic(object):
    def __init__(self, topic_name, msg_type, queue_size=0, use_serializer=False):
        super(FrameworkTopic, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.use_serializer = use_serializer

    def register_subscriber(self, callback=None, callback_args=None):
        sub_args = {"topic": self.topic_name}

        if callback_args is not None:
            sub_args.update(callback_args)

        comm.register_subscriber(self.topic_name, self.msg_type, callback, sub_args)

    def get_publisher(self):
        return FrameworkTopicPublisher(self.topic_name, self.msg_type, self.queue_size, self.use_serializer)


class FrameworkTopicPublisher(object):
    def __init__(self, topic_name, msg_type, queue_size=0, use_serializer=False):
        super(FrameworkTopicPublisher, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.use_serializer = use_serializer

        self._publisher \
            = comm.get_publisher(self.topic_name,
                                 self.msg_type,
                                 queue_size=self.queue_size)

    def publish(self, msg):
        logger.debug("Publishing msg to topic [{}]".format(self.topic_name))

        if self.use_serializer:
            outbound_msg = MsgSerializer.serialize(msg)
        else:
            outbound_msg = msg

        comm.publish_message(self._publisher, outbound_msg)


class MsgSerializer(object):
    @staticmethod
    def serialize(obj):
        return dumps(obj)

    @staticmethod
    def deserialize(serialized_obj):
        return loads(serialized_obj)


built_in_topics = {
    "selected_behaviors": FrameworkTopic("selected_behaviors", String, use_serializer=True),
    "candidate_behaviors": FrameworkTopic("candidate_behaviors", String, use_serializer=True),
    "global_broadcast": FrameworkTopic("global_broadcast", String, use_serializer=True),
    "episodes": FrameworkTopic("episodes", String, use_serializer=True),
    "workspace_cues": FrameworkTopic("workspace_cues", String, use_serializer=True),
    "workspace_coalitions": FrameworkTopic("workspace_coalitions", String, use_serializer=True),
    "detected_features": FrameworkTopic("detected_features", String, use_serializer=True),
    "percepts": FrameworkTopic("percepts", String, use_serializer=True),
    "spatial_maps": FrameworkTopic("spatial_maps", String, use_serializer=True),
    "dorsal_stream": FrameworkTopic("dorsal_stream", String, use_serializer=True),
    "ventral_stream": FrameworkTopic("ventral_stream", String, use_serializer=True),
}
