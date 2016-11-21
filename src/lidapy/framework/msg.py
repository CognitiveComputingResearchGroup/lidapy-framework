from lidapy.framework.shared import FrameworkObject
from lidapy.util.functions import generate_random_name


class FrameworkTopic(FrameworkObject):
    def __init__(self, topic_name=None, msg_type=None):
        super(FrameworkTopic, self).__init__()

        if topic_name is None:
            topic_name = generate_random_name(prefix="Topic_", length=16)

        self.topic_name = topic_name
        self.msg_type = msg_type

        self.publisher = None
        self.subscriber = None

    def create_publisher(self, max_queue_size=None, preprocessor=None):
        """ Creates a message publisher for this topic.

        :param max_queue_size: the maximum size for the send queue before messages are dropped in FIFO order.
        :param preprocessor: a callable that will be invoked prior to publishing a message to the message queue.
        :return: a FrameworkTopicPublisher
        """
        if self.publisher is None:
            self.logger.info("Initializing publisher for topic {}".format(self.topic_name))

            self.publisher \
                = self.ipc_proxy.get_publisher(topic_name=self.topic_name,
                                               msg_type=self.msg_type,
                                               max_queue_size=max_queue_size,
                                               preprocessor=preprocessor)
        else:
            raise Exception("Publisher already added for topic {}".format(self.topic_name))

        return self.publisher

    def create_subscriber(self, max_queue_size=None, postprocessor=None):
        """ Creates a subscriber for this topic.  The subscriber will automatically retrieve
        messages and place them on a local queue to be retrieved later.  A postprocessor
        callable can be specified to perform operations on the incoming messages after
        retrieving them from the message queue.

        :param max_queue_size: the maximum size of the receive queue before messages are dropped in FIFO order.
        :param postprocessor: a callable that will be invoked after receiving a message from the message queue.
        :return: a FrameworkTopicSubscriber
        """
        if self.subscriber is None:
            self.logger.info("Initializing subscriber for topic {}".format(self.topic_name))

            self.subscriber = \
                self.ipc_proxy.get_subscriber(topic_name=self.topic_name,
                                              msg_type=self.msg_type,
                                              max_queue_size=max_queue_size,
                                              postprocessor=postprocessor)
        else:
            raise Exception("Subscriber already added for topic {}".format(self.topic_name))

        return self.subscriber
