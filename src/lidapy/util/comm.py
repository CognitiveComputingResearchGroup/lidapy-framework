from abc import ABCMeta, abstractmethod

from datetime import datetime
from time import sleep
from collections import deque

from cPickle import dumps, loads  # Object Serialization / Deserialization

from lidapy.framework.process import FrameworkThread
from lidapy.framework.shared import FrameworkObject

# ROS specific imports
import rospy
from lidapy.util.functions import generate_random_name
from std_msgs.msg import String


class FrameworkCommunicationProxy(object):
    __metaclass__ = ABCMeta

    def __init__(self, name, *args, **kwargs):
        self.name = name

        self._is_shutting_down = False

    @abstractmethod
    def initialize_node(self, name, log_level):
        pass

    @abstractmethod
    def get_publisher(self, topic_name, msg_type, max_queue_size, preprocessor):
        pass

    @abstractmethod
    def get_subscriber(self, topic_name, msg_type, max_queue_size, postprocessor):
        pass

    @abstractmethod
    def get_service(self, service_name, service_class, callback):
        pass

    @abstractmethod
    def get_service_proxy(self, service_name, service_class):
        pass

    def force_shutdown(self):
        self._is_shutting_down = True

    def is_shutting_down(self):
        return self._is_shutting_down

    def wait(self, rate_in_hz):
        sleep(1.0 / rate_in_hz)

    def get_current_time(self):
        return datetime.now()


class RosCommunicationProxy(FrameworkCommunicationProxy):
    def __init__(self, *args, **kwargs):
        super(RosCommunicationProxy, self).__init__("RosCommunicationProxy", *args, **kwargs)

    def initialize_node(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)

    def get_publisher(self, topic_name, msg_type, max_queue_size=None, preprocessor=None):
        return RosTopicPublisher(topic_name=topic_name,
                                 msg_type=msg_type,
                                 queue_size=max_queue_size,
                                 preprocessor=preprocessor)

    def get_subscriber(self, topic_name, msg_type=None, max_queue_size=None, postprocessor=None):
        return RosTopicSubscriber(topic_name=topic_name,
                                  msg_type=msg_type,
                                  queue_size=max_queue_size,
                                  postprocessor=postprocessor)

    def get_service(self, service_name, service_class, callback):
        return rospy.Service(service_name, service_class, callback)

    def get_service_proxy(self, service_name, service_class):
        return rospy.ServiceProxy(service_name, service_class)

    def is_shutting_down(self):
        if super(RosCommunicationProxy, self).is_shutting_down():
            return True
        elif rospy.is_shutdown():
            return True

        return False

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


class LocalCommunicationProxy(FrameworkCommunicationProxy):
    def __init__(self, *args, **kwargs):
        super(LocalCommunicationProxy, self).__init__("LocalCommunicationProxy", *args, **kwargs)

        # A dictionary of queues
        # {
        #  topic_name_1 : msg_queue_1,
        #  topic_name_2 : msg_queue_2,
        # }
        self.msg_queues = {}

    def initialize_node(self, name, log_level):
        # No-op
        pass

    def get_publisher(self, topic_name, msg_type=None, max_queue_size=None, preprocessor=None):
        if topic_name not in self.msg_queues:
            self.msg_queues[topic_name] = LocalMessageQueue(topic_name, msg_type)

        return LocalTopicPublisher(self.msg_queues[topic_name], queue_size=max_queue_size, preprocessor=preprocessor)

    def get_subscriber(self, topic_name, msg_type=None, max_queue_size=None, postprocessor=None):
        if topic_name not in self.msg_queues:
            self.msg_queues[topic_name] = LocalMessageQueue(topic_name, msg_type)

        topic_listener \
            = LocalTopicSubscriber(self.msg_queues[topic_name],
                                   queue_size=max_queue_size,
                                   postprocessor=postprocessor)

        # Start thread listening on topic
        topic_listener.start()

        return topic_listener

    def get_service(self, service_name, service_class, callback):
        raise NotImplementedError

    def get_service_proxy(self, service_name, service_class):
        raise NotImplementedError


class FrameworkTopicPublisher(FrameworkObject):
    __metaclass__ = ABCMeta

    def __init__(self, topic_name, msg_type, queue_size, preprocessor):
        super(FrameworkTopicPublisher, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.preprocessor = preprocessor

    @abstractmethod
    def publish(self, msg):
        pass


class FrameworkTopicSubscriber(FrameworkObject):
    __metaclass__ = ABCMeta

    def __init__(self, topic_name, msg_type, queue_size, postprocessor):
        super(FrameworkTopicSubscriber, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.postprocessor = postprocessor

        # Temporary location to store messages received from topic
        self._receive_queue = deque(maxlen=queue_size)

    def get_next_msg(self):
        if len(self._receive_queue) == 0:
            return None

        return self._receive_queue.pop()

    @property
    def msg_count(self):
        return len(self._receive_queue)


class RosTopicPublisher(FrameworkTopicPublisher):
    def __init__(self, topic_name, msg_type=String, queue_size=None, preprocessor=None):
        super(RosTopicPublisher, self).__init__(topic_name, msg_type, queue_size, preprocessor)

        if preprocessor is None and msg_type is String:
            def default_preprocessor(msg):
                return RosMsgUtils.wrap(MsgUtils.serialize(msg), msg_type, "data")

            self.preprocessor = default_preprocessor

        self._publisher = rospy.Publisher(name=self.topic_name,
                                          data_class=self.msg_type,
                                          queue_size=self.queue_size)

    def publish(self, msg):
        if msg is None:
            raise Exception("Attempted to publish msg on topic = {} with value = None".format(self.topic_name))

        if callable(self.preprocessor):
            msg = self.preprocessor(msg)

        self._publisher.publish(msg)


class RosTopicSubscriber(FrameworkTopicSubscriber):
    def __init__(self, topic_name, msg_type=String, queue_size=None, postprocessor=None):
        super(RosTopicSubscriber, self).__init__(topic_name, msg_type, queue_size, postprocessor)

        if postprocessor is None and msg_type is String:
            def default_postprocessor(msg):
                return MsgUtils.deserialize(RosMsgUtils.unwrap(msg, "data"))

            self.postprocessor = default_postprocessor

        self._subscriber = rospy.Subscriber(name=self.topic_name,
                                            data_class=self.msg_type,
                                            callback=self._listener_func,
                                            callback_args=None,
                                            queue_size=self.queue_size)

    def _listener_func(self, msg, *args):
        if callable(self.postprocessor):
            msg = self.postprocessor(msg)

        self._receive_queue.append(msg)


class LocalTopicPublisher(FrameworkTopicPublisher):
    def __init__(self, msg_queue, queue_size=None, preprocessor=None):
        super(LocalTopicPublisher, self).__init__(topic_name=msg_queue.name,
                                                  msg_type=msg_queue.msg_type,
                                                  queue_size=queue_size,
                                                  preprocessor=preprocessor)

        self.msg_queue = msg_queue

    def publish(self, msg):
        if msg is None:
            raise Exception(
                "Attempted to publish msg on topic = {} with value = None".format(
                    self.msg_queue.topic_name))

        if callable(self.preprocessor):
            msg = self.preprocessor(msg)

        self.msg_queue.push(msg)


class LocalTopicSubscriber(FrameworkTopicSubscriber):
    def __init__(self, msg_queue, queue_size=None, postprocessor=None):
        super(LocalTopicSubscriber, self).__init__(topic_name=msg_queue.name,
                                                   msg_type=msg_queue.msg_type,
                                                   queue_size=queue_size,
                                                   postprocessor=postprocessor)

        self.msg_queue = msg_queue

        self._thread = FrameworkThread(name="topic_listener", callback=self._listener_func)
        self._thread.daemon = True

    def _listener_func(self, *args):
        if len(self.msg_queue) > 0:
            msg = self.msg_queue.pop()

            if callable(self.postprocessor):
                msg = self.postprocessor(msg)

            self._receive_queue.append(msg)

    def start(self):
        self._thread.start()


class LocalMessageQueue(FrameworkObject):
    def __init__(self, name=None, msg_type=None, max_queue_size=None):
        super(LocalMessageQueue, self).__init__()

        if name is None:
            name = generate_random_name(prefix="LocalQueue_", length=16)

        self.name = name
        self.msg_type = msg_type
        self.max_queue_size = max_queue_size

        self._queue = deque(maxlen=max_queue_size)

    def push(self, msg):
        if msg is not None:
            self._queue.append(msg)

    def pop(self):
        if len(self._queue) != 0:
            return self._queue.popleft()

    def peek(self):
        return self._queue[-1]

    def __len__(self):
        return len(self._queue)


class MsgUtils(object):
    @staticmethod
    def serialize(obj):
        return dumps(obj)

    @staticmethod
    def deserialize(serialized_obj):
        return loads(serialized_obj)


class RosMsgUtils(object):
    @staticmethod
    def wrap(obj, cls, prop):
        wrapper_cls = cls()

        setattr(wrapper_cls, prop, obj)

        return wrapper_cls

    @staticmethod
    def unwrap(obj, prop):
        if hasattr(obj, prop):
            return getattr(obj, prop)
