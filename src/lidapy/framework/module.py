from abc import ABCMeta, abstractmethod

from lidapy.framework.process import FrameworkProcess
from lidapy.framework.service import FrameworkService


class FrameworkModule(FrameworkProcess):
    __metaclass__ = ABCMeta

    """ The FrameworkModule class is used to sub-divide an agent into high-level processing components called modules.

    During initialization, the FrameworkModule is registered as a ros node and publishers/subscribers/services
    are registered with the ros master (as necessary).  The call method is invoked at regular intervals (as determined
    by the \"rate_in_hz\" parameter).  Care should be taken to ensure that the call method does not have any
    long-running (blocking) operations as this may cause the module to become unresponsive.

    Each FrameworkModule must run as a separate process.

    """

    def __init__(self, **kwargs):
        super(FrameworkModule, self).__init__(self.get_module_name(), **kwargs)

        # A dictionary of FrameworkTopics
        #
        # format:
        # {
        #     TopicName1 : FrameworkTopic1,
        #     TopicName2 : FrameworkTopic2,
        # }
        self.topics = {}

        # A dictionary of FrameworkTopicPublishers
        #
        # format:
        # {
        #     TopicName1 : FrameworkTopicPublisher1,
        #     TopicName2 : FrameworkTopicPublisher2,
        # }
        self.publishers = {}

        # A dictionary of FrameworkTopicSubscribers
        #
        # format:
        # {
        #     TopicName1 : FrameworkTopicSubscriber1,
        #     TopicName2 : FrameworkTopicSubscriber2,
        # }
        self.subscribers = {}

        # A dictionary of FrameworkServices
        #
        # format:
        # {
        #     ServiceName1 : FrameworkService1,
        #     ServiceName2 : FrameworkService2,
        # }
        self.services = {}

        # A list of FrameworkTasks
        #
        # format:
        # [
        #     FrameworkTask1,
        #     FrameworkTask2,
        # ]
        self.background_tasks = []

    @classmethod
    def get_module_name(cls):
        raise NotImplemented("FrameworkModule must implement get_module_name classmethod")

    def initialize(self):
        """A template method that initializes the FrameworkModule instance by invoking a set of standard methods.

        These standard method implementations can be overridden in subclasses of FrameworkModule
        to provide custom behaviors (e.g., adding publishers, subscribers, services, and background tasks).

        This method can be overridden to customize module initialization.

        :return: None
        """
        super(FrameworkModule, self).initialize()

        self.launch_background_tasks()

    def add_service(self, svc_name, svc_msg_class, callback):
        """ Registers a service with the specified request/reply message classes and callback methods.

        :param svc_name: the name of the service.
        :param svc_msg_class: the ros message class that contains the request/reply classes.
        :param callback: a callback method that will be invoked when a request is made to this service.
        :return: None
        """
        decorated_svc_name = "{}/{}".format(self.name, svc_name)
        self.services[decorated_svc_name] = FrameworkService(decorated_svc_name, svc_msg_class, callback)

    def add_background_task(self, task):
        """ Registers a background task associated with this framework module.

        Background tasks are used to schedule the concurrent execution of methods
        at regular intervals.  These methods are executed independent of the usual
        FrameworkModule flow of control.

        :param task: a FrameworkTask to register as a background task.
        :return: None
        """
        self.logger.info("Adding background task ({})".format(task.name))
        self.background_tasks.append(task)

    def add_publishers(self, topics):
        """ Adds publishers to this FrameworkModule to enable asynchronous transmission of messages over topics.

        This method must be overridden to add publishers to this FrameworkModule.

        :param: a list of FrameworkTopics
        :return: None
        """
        self.publishers \
            = {topic.topic_name: topic.create_publisher() for topic in topics}

    def add_subscribers(self, topics):
        """ Adds subscribers to this FrameworkModule to enable asynchronous retrieval of messages over topics.

        This method must be overridden to add subscribers to this FrameworkModule.

        :param: a list of FrameworkTopics
        :return: None
        """
        self.subscribers \
            = {topic.topic_name: topic.create_subscriber() for topic in topics}

    def add_services(self):
        """ Adds services to this FrameworkModule to enable synchronous (RPC style) communication.

        This method must be overridden to add subscribers to this FrameworkModule.

        :return: None
        """
        pass

    def add_background_tasks(self):
        """ Adds background tasks to this FrameworkModule to enable the concurrent execution of FrameworkTasks.

        This method must be overridden to add background tasks to this FrameworkModule.

        :return: None
        """
        pass

    def launch_background_tasks(self):
        for t in self.background_tasks:
            self.logger.info("Starting background task \"{}\"".format(t.name))
            t.start()

    @abstractmethod
    def call(self):
        """ The entry-point for FrameworkModule execution.

        This method is intended to be invoked at regular intervals, where the interval length is
        determined by the \"rate_in_hz\" agent configuration parameter.

        This method must be overridden.

        Steps in module processing, such as the consumption of messages from ros topics and the publication
        of messages to ros topics, should occur within this method.

        Care should be taken not to execute long running operations in a blocking mode within
        the call method as that will result in the module becoming unresponsive.  If long-running
        operations are required they should be scheduled in a separate background task or using
        non-blocking mode.

        :return: None
        """
        pass
