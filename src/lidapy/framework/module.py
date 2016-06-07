from collections import deque

from lida.srv import decayModule, cueModule

from lidapy.framework.agent import AgentConfig
from lidapy.framework.process import FrameworkProcess
from lidapy.framework.service import FrameworkService
from lidapy.util import logger


class FrameworkModule(FrameworkProcess):
    def __init__(self, name, **kwargs):
        super(FrameworkModule, self).__init__(name, **kwargs)

        self.name = name

        self.cueable = kwargs.get("cueable", False)
        self.decayable = kwargs.get("decayable", False)

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

        # A dictionary of FrameworkServices
        #
        # format:
        # {
        #     ServiceName1 : FrameworkService1,
        #     ServiceName2 : FrameworkService2,
        # }
        self.services = {}

        # A dictionary of message queues.
        #
        # format:
        # {
        #     TopicName1 : Deque1,
        #     TopicName2 : Deque2,
        # }
        self.received_msgs = {}

        self._config = None

    # Can be override to customize module initialization.  Note: When running
    # the module in a separate process, initialize must be called after forking
    # (i.e., starting) the process
    def initialize(self):
        super(FrameworkModule, self).initialize()

        self.add_publishers()
        self.add_subscribers()
        self.add_services()

    @property
    def config(self):
        if self._config is None:
            self._config = AgentConfig()

        return self._config

    @property
    def logger(self):
        return logger

    # A default callback for topic subscribers. This can be overridden; however, if
    # this method is overrideen then get_next_msg should also be overridden.
    def receive_msg(self, msg, args):
        topic_name = args["topic"]

        self.logger.debug("Receiving message on topic {}.  Message = {}".format(topic_name, msg))

        if topic_name is not None:
            msg_queue = self.received_msgs[topic_name]
            msg_queue.append(msg)

    # A default implementation for retrieving messages for a topic.  This
    # implementation assumes the default callback "receive_msg"
    def get_next_msg(self, topic_name):
        msg_queue = self.received_msgs[topic_name]

        next_msg = None
        if len(msg_queue) == 0:
            self.logger.debug("Message queue is empty for topic {}".format(topic_name))
        else:
            next_msg = msg_queue.popleft()

        return next_msg

    def add_publisher(self, topic):
        self.logger.info("Adding publisher for topic {}".format(topic.topic_name))

        self.publishers[topic.topic_name] = topic.get_publisher()

    def add_subscriber(self, topic, callback=None, callback_args=None):
        self.logger.info("Adding subscriber for topic {}".format(topic.topic_name))

        # Initial message queue
        max_queue_size = self.config.get_type_or_global_param(self.name, "max_queue_size", 10)
        self.received_msgs[topic.topic_name] = deque(maxlen=max_queue_size)

        if callback is None:
            callback = self.receive_msg

        topic.register_subscriber(callback, callback_args)

    def add_service(self, svc_name, svc_msg_class, callback):
        decorated_svc_name = "{}/{}".format(self.name, svc_name)
        self.service = FrameworkService(decorated_svc_name, svc_msg_class, callback)

    # This method must be overridden
    def add_publishers(self):
        pass

    # This method must be overridden
    def add_subscribers(self):
        pass

    def add_services(self):
        self.logger.debug("Adding services")

        if self.cueable:
            self.add_service("cue", cueModule, self.receive_cue_request)

        if self.decayable:
            self.add_service("decay", decayModule, self.receive_decay_request)

    # This method must be overridden
    def call(self):
        super(FrameworkModule, self).call()

    def learn(self):
        self.logger.debug("Learning")

    # TODO: Need to add decay strategies
    def decay(self, decay_strategy):
        self.logger.debug("Decaying")

    def receive_decay_request(self, request):
        self.logger.debug("Receiving decay request: {}".format(request))

        if request is None:
            return

        n = request.n
        strategy = request.strategy

        for i in xrange(1..n):
            self.decay(strategy)

    def receive_cue_request(self, request):
        self.logger.debug("Receiving cue request: {}".format(request))

        if request is None:
            return
