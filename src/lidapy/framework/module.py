from collections import deque

from lidapy_rosdeps.srv import decayModule, cueModule

from lidapy.framework.process import FrameworkProcess
from lidapy.framework.service import FrameworkService
from lidapy.util import logger


class FrameworkModule(FrameworkProcess):
    """ The FrameworkModule class is used to sub-divide an agent into high-level processing components called modules.

    During initialization, the FrameworkModule is registered as a ros node and publishers/subscribers/services
    are registered with the ros master (as necessary).  The call method is invoked at regular intervals (as determined
    by the \"rate_in_hz\" parameter).  Care should be taken to ensure that the call method does not have any
    long-running (blocking) operations as this may cause the module to become unresponsive.

    Each FrameworkModule must run as a separate process.

    """

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

    def initialize(self):
        """A template method that initializes the FrameworkModule instance by invoking a set of standard methods.

        These standard method implementations can be overridden in subclasses of FrameworkModule
        to provide custom behaviors (e.g., adding publishers, subscribers, and services).  This method
        can be overridden to customize module initialization.

        :return: None
        """
        super(FrameworkModule, self).initialize()

        self.add_publishers()
        self.add_subscribers()
        self.add_services()

    def receive_msg(self, msg, args):
        """ A default callback for topic subscribers used for receiving messages.

        This method can be overridden to provide different callback behavior; however, if
        overridden, the method get_next_msg (which depends on this method) may also need
        to be overridden to ensure consistent behavior.

        :param msg: a message received over a ros topic
        :param args: a list of arguments to the callback that were set when the callback
                     was registered for the subscriber.
        :return: None
        """
        topic_name = args["topic"]

        logger.debug("Receiving message on topic {}.  Message = {}".format(topic_name, msg))

        if topic_name is not None:
            msg_queue = self.received_msgs[topic_name]
            msg_queue.append(msg)

    def get_next_msg(self, topic_name):
        """ Returns a message (in FIFO order) for the specified topic or None if no messages are available.

        This is a default implementation for retrieving messages over a topic.  This implementation assumes
        that the default callback (i.e., default receive_msg implementation) was registered with the
        subscriber.

        :param topic_name: the name of the topic for which a previously retrieved message will be returned.
        :return: a message for the specified topic_name.
        """
        msg_queue = self.received_msgs[topic_name]

        next_msg = None
        if len(msg_queue) == 0:
            logger.debug("Message queue is empty for topic {}".format(topic_name))
        else:
            next_msg = msg_queue.popleft()

        return next_msg

    def add_publisher(self, topic):
        """ Registers a publisher for a topic.

        Publishers are used to asynchronously transmit messages over a ros topic.

        :param topic: the framework topic for which a publisher will be created.
        :return: None
        """
        logger.info("Adding publisher for topic {}".format(topic.topic_name))

        self.publishers[topic.topic_name] = topic.get_publisher()

    def add_subscriber(self, topic, callback=None, callback_args=None):
        """ Registers a subscriber for a topic with the specified callback method.

        :param topic: a FrameworkTopic for this subscriber
        :param callback: a callback method invoked when a new messages is available
        :param callback_args: a list of arguments will be provided to the callback method when invoked
        :return: None
        """
        logger.info("Adding subscriber for topic {}".format(topic.topic_name))

        # Initial message queue
        max_queue_size = self.config.get_type_or_global_param(self.name, "max_queue_size", 10)
        self.received_msgs[topic.topic_name] = deque(maxlen=max_queue_size)

        if callback is None:
            callback = self.receive_msg

        topic.register_subscriber(callback, callback_args)

    def add_service(self, svc_name, svc_msg_class, callback):
        """ Registers a service with the specified request/reply message classes and callback methods.

        :param svc_name: the name of the service.
        :param svc_msg_class: the ros message class that contains the request/reply classes.
        :param callback: a callback method that will be invoked when a request is made to this service.
        :return: None
        """
        decorated_svc_name = "{}/{}".format(self.name, svc_name)
        self.service = FrameworkService(decorated_svc_name, svc_msg_class, callback)

    # This method must be overridden
    def add_publishers(self):
        """ Adds publishers to this FrameworkModule to enable asynchronous transmission of messages over topics.

        This method must be overridden to add publishers to this FrameworkModule.

        :return: None
        """
        pass

    def add_subscribers(self):
        """ Adds subscribers to this FrameworkModule to enable asynchronous retrieval of messages over topics.

        This method must be overridden to add subscribers to this FrameworkModule.

        :return: None
        """
        pass

    def add_services(self):
        """ Adds services to this FrameworkModule to enable synchronous (RPC style) communication.

        This method must be overridden to add subscribers to this FrameworkModule.

        Note that the default implementation will add a cue and decay service based on arguments to the class
        initializer.

        :return: None
        """
        logger.debug("Adding services")

        if self.cueable:
            self.add_service("cue", cueModule, self.receive_cue_request)

        if self.decayable:
            self.add_service("decay", decayModule, self.receive_decay_request)

    def call(self):
        """ The entry-point for FrameworkModule execution.

        This method is intended to be invoked at regular intervals, where the interval length is
        determined by the \"rate_in_hz\" agent configuration parameter.

        This method must be overridden.

        Steps in module processing, such as the consumption of messages from ros topics and the publication
        of messages to ros topics, should occur within this method.

        Care should be taken not to execute long running operations in a blocking mode within
        the call method as that will result in the module becoming unresponsive.  If long-running
        operations are required they should be invoked in non-blocking mode.

        :return: None
        """
        super(FrameworkModule, self).call()

    def learn(self):
        """ Updates the representations within this module based on a received global broadcast.

        This method must be overridden.

        :return: None
        """
        logger.debug("Learning")

    def decay(self, decay_strategy):
        """ Decays the representations within this module based on the supplied decay strategy.

        :param decay_strategy: a decay strategy that controls the rate of module decay.
        :return: None
        """
        logger.debug("Decaying")

    def receive_decay_request(self, request):
        logger.debug("Receiving decay request: {}".format(request))

        if request is None:
            return

        n = request.n
        strategy = request.strategy

        for i in xrange(1..n):
            self.decay(strategy)

    def receive_cue_request(self, request):
        logger.debug("Receiving cue request: {}".format(request))

        if request is None:
            return
