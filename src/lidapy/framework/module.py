from Queue import Queue, Empty, Full

from lidapy.framework.agent import AgentConfig
from lidapy.framework.process import FrameworkProcess
from lidapy.util import logger


class FrameworkModule(FrameworkProcess):

    def __init__(self, module_name):
        super(FrameworkModule, self).__init__(module_name)

        self.module_name = module_name

        # A dictionary of FrameworkTopics
        #
        # format:
        # {
        #     TopicName1 : FrameworkTopic1,
        #     TopicName2 : FrameworkTopic2,
        # }
        self.topics = {}

        # A dictionary of FrameworkTopics
        #
        # format:
        # {
        #     TopicName1 : FrameworkTopicPublisher1,
        #     TopicName2 : FrameworkTopicPublisher2,
        # }
        self.publishers = {}

        # A dictionary of message queues.
        #
        # format:
        # {
        #     TopicName1 : Queue1,
        #     TopicName2 : Queue2,
        # }
        self.received_msgs = {}

        self._config = None

        self.add_publishers()
        self.add_subscribers()

    @property
    def config(self):
        if self._config is None:
            self._config = AgentConfig()

        return self._config

    @property
    def logger(self):
        return logger

    # A default callback for topic subscribers.
    def receive_msg(self, msg, args):
        topic_name = args["topic"]

        self.logger.debug("Receiving message on topic {}.  Message = {}".format(topic_name, msg))

        if topic_name is not None:
            msg_queue = self.received_msgs[topic_name]

            try:
                msg_queue.put_nowait(msg)
            except Full:
                self.logger.warn("Message queue is full for topic {}".format(topic_name))

    # A default implementation for retrieving messages for a topic.  This
    # implementation assumes the default callback "receive_msg"
    def get_next_msg(self, topic_name):
        msg_queue = self.received_msgs[topic_name]

        next_msg = None
        try:
            next_msg = msg_queue.get_nowait()
        except Empty:
            self.logger.debug("Message queue is empty for topic {}".format(topic_name))

        return next_msg

    def add_publisher(self, topic):
        self.logger.info("Adding publisher for topic {}".format(topic.topic_name))

        self.publishers[topic.topic_name] = topic.get_publisher()

    def add_subscriber(self, topic, callback=None, callback_args=None):
        self.logger.info("Adding subscriber for topic {}".format(topic.topic_name))

        self.received_msgs[topic.topic_name] = Queue()

        if callback is None:
            callback = self.receive_msg

        topic.register_subscriber(callback, callback_args)

    # This method must be overridden
    def add_publishers(self):
        pass

    # This method must be overridden
    def add_subscribers(self):
        pass

    # This method must be overridden
    def advance(self):
        pass
