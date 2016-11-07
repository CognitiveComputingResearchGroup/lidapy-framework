from abc import ABCMeta, abstractmethod
from datetime import datetime
from time import sleep

import rospy


class AbstractCommunicationProxy:
    __metaclass__ = ABCMeta

    def __init__(self, name, *args, **kwargs):
        self.name = name

    @abstractmethod
    def initialize_node(self, name, log_level):
        pass

    @abstractmethod
    def get_publisher(self, topic, msg_type, queue_size):
        pass

    @abstractmethod
    def get_subscriber(self, topic, msg_type, callback, callback_args):
        pass

    @abstractmethod
    def get_service(self, service_name, service_class, callback):
        pass

    @abstractmethod
    def get_service_proxy(self, service_name, service_class):
        pass

    @abstractmethod
    def is_shutting_down(self):
        pass

    def wait(self, rate_in_hz):
        sleep(1.0 / rate_in_hz)

    def get_current_time(self):
        return datetime.now()


class RosCommunicationProxy(AbstractCommunicationProxy):
    def __init__(self, *args, **kwargs):
        super(RosCommunicationProxy, self).__init__("RosCommunicationProxy", *args, **kwargs)

    def initialize_node(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)

    def get_publisher(self, topic, msg_type, queue_size=0):
        return rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def get_subscriber(self, topic, msg_type, callback, callback_args):
        return rospy.Subscriber(topic, msg_type, callback=callback, callback_args=callback_args)

    def get_service(self, service_name, service_class, callback):
        return rospy.Service(service_name, service_class, callback)

    def get_service_proxy(self, service_name, service_class):
        return rospy.ServiceProxy(service_name, service_class)

    def is_shutting_down(self):
        return rospy.is_shutdown()

    def wait(self, rate):
        waiter = rospy.Rate(rate)
        waiter.sleep()

    def get_current_time(self):
        return rospy.get_rostime()


class ParameterService(object):
    def get_param(self, param_type, param_name, default_value=None):
        fully_qualified_name = self.get_fully_qualified_param_name(param_type, param_name)
        return rospy.get_param(fully_qualified_name, default_value)

    def set_param(self, param_type, param_name, param_value):
        fully_qualified_name = self.get_fully_qualified_param_name(param_type, param_name)
        return rospy.set_param(fully_qualified_name, param_value)

    def get_fully_qualified_param_name(self, param_type, param_name):
        return "/".join(["lida", param_type, param_name])


class StubCommunicationProxy(AbstractCommunicationProxy):
    def __init__(self, *args, **kwargs):
        super(StubCommunicationProxy, self).__init__("StubCommunicationProxy", *args, **kwargs)

    def initialize_node(self, name, log_level):
        pass

    def get_publisher(self, topic, msg_type, queue_size=0):
        pass

    def get_subscriber(self, topic, msg_type, callback, callback_args):
        pass

    def get_service(self, service_name, service_class, callback):
        pass

    def get_service_proxy(self, service_name, service_class):
        pass

    def is_shutting_down(self):
        return False
