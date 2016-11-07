from cPickle import dumps, loads  # Object Serialization / Deserialization

from lidapy.framework.shared import FrameworkObject
from std_msgs.msg import String


class FrameworkTopic(FrameworkObject):
    def __init__(self, topic_name, msg_type=String, queue_size=0, use_serializer=False):
        super(FrameworkTopic, self).__init__()

        self.msg_type = msg_type
        self.queue_size = queue_size
        self.topic_name = topic_name
        self.use_serializer = use_serializer

        # self.logger = FrameworkDependency("logger").resolve()
        # self.comm_proxy = FrameworkDependency("ipc_proxy").resolve()

    def register_subscriber(self, callback=None, callback_args=None):
        sub_args = {"topic": self.topic_name}

        if callback_args is not None:
            sub_args.update(callback_args)

        self.ipc_proxy.get_subscriber(self.topic_name, self.msg_type, callback, sub_args)

    def get_publisher(self):
        return FrameworkTopicPublisher(self.topic_name, self.msg_type, self.queue_size, self.use_serializer)


class FrameworkTopicPublisher(FrameworkObject):
    def __init__(self, topic_name, msg_type, queue_size=0, use_serializer=False):
        super(FrameworkTopicPublisher, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.use_serializer = use_serializer

        # self.logger = FrameworkDependency("logger").resolve()
        # self.comm_proxy = FrameworkDependency("ipc_proxy").resolve()

        self._publisher \
            = self.ipc_proxy.get_publisher(self.topic_name,
                                           self.msg_type,
                                           queue_size=self.queue_size)

    def publish(self, msg):
        self.logger.debug("Publishing msg to topic [{}]".format(self.topic_name))

        if self.use_serializer:
            outbound_msg = MsgSerializer.serialize(msg)
        else:
            outbound_msg = msg

        self._publisher.publish(outbound_msg)


class MsgSerializer(object):
    @staticmethod
    def serialize(obj):
        return dumps(obj)

    @staticmethod
    def deserialize(serialized_obj):
        return loads(serialized_obj)
