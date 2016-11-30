from lidapy.framework.process import FrameworkProcess
from lidapy.framework.service import FrameworkService


class FrameworkModule(FrameworkProcess):
    """ The FrameworkModule class is used to sub-divide an agent into high-level processing components called modules.
    """

    def __init__(self, **kwargs):
        super(FrameworkModule, self).__init__(self.get_module_name())

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

        # A list of FrameworkTasks
        #
        # format:
        # [
        #     TaskName1 : FrameworkBackgroundTask1,
        #     TaskName2 : FrameworkBackgroundTask2,
        # ]
        self.background_tasks = {}

        # A dictionary of FrameworkServices
        #
        # format:
        # {
        #     ServiceName1 : FrameworkService1,
        #     ServiceName2 : FrameworkService2,
        # }
        self.services = {}

    def add_service(self, svc_name, svc_msg_class, callback):
        """ Registers a service with the specified request/reply message classes and callback methods.

        :param svc_name: the name of the service.
        :param svc_msg_class: the ros message class that contains the request/reply classes.
        :param callback: a callback method that will be invoked when a request is made to this service.
        :return: None
        """
        decorated_svc_name = "{}/{}".format(self.name, svc_name)
        self.services[decorated_svc_name] = FrameworkService(decorated_svc_name, svc_msg_class, callback)

    def add_publishers(self, topics):
        """ Creates publishers for the requested topics. (See FrameworkTopicPublisher.)

        :param: a list of FrameworkTopics
        :return: None
        """
        self.publishers \
            = {topic.topic_name: topic.create_publisher() for topic in topics}

    def add_subscribers(self, topics):
        """ Creates subscribers for the requested topics. (See FrameworkTopicSubscriber.)

        :param: a list of FrameworkTopics
        :return: None
        """
        self.subscribers \
            = {topic.topic_name: topic.create_subscriber() for topic in topics}

    def add_background_tasks(self, tasks):
        """ Adds background tasks to this FrameworkModule to enable. (See FrameworkBackgroundTask.)

        :param: a list of FrameworkBackgroundTasks
        :return: None
        """
        self.background_tasks = {task.name: task for task in tasks}

    def launch_background_tasks(self):
        """ Starts each of the FrameworkBackgroundTasks registered with the module.

        :return: None
        """
        for task in self.background_tasks.itervalues():
            task.start()

    @classmethod
    def get_module_name(cls):
        raise NotImplemented("Module must implement get_module_name classmethod")

    def initialize(self):
        """Initializes this FrameworkModule.

        This method can be overridden to customize module initialization; however, any derived class
        that overrides this method should call the super class's initialize method.

        :return: None
        """
        super(FrameworkModule, self).initialize()

    def finalize(self):
        """Finalizes this FrameworkModule.

        This method can be overridden to customize module finalization; however, any derived class
        that overrides this method should call the super class's finalize method.

        :return: None
        """
        super(FrameworkModule, self).finalize()

    def start(self):
        """Starts the execution of this FrameworkModule.

        :return: None
        """
        super(FrameworkProcess, self).start()

    def update_status(self):
        """Update the status of this FrameworkModule.

        :return: None
        """
        super(FrameworkModule, self).update_status()

        for task in self.background_tasks.itervalues():
            if task.status is self.ERROR:
                self.status = self.ERROR
