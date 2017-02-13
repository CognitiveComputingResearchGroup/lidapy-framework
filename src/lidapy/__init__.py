from __future__ import print_function

import ConfigParser
import abc
import collections
import copy
import datetime
import multiprocessing
import sys
import threading
import time
import traceback
from os import getpid

import rospy
import std_msgs.msg as std_msgs
from lidapy.util import MsgUtils
from lidapy.util import RosMsgUtils
from lidapy.util import create_class_instance
from lidapy.util import generate_random_name

# from lidapy_rosdeps.srv import GenericService

# Internal LidaPy Globals
_var = collections.namedtuple('lidapy_var', ['config', 'logger', 'ipc'])

LOG_LEVEL_DEBUG = 0
LOG_LEVEL_INFO = 1
LOG_LEVEL_WARN = 2
LOG_LEVEL_ERROR = 3
LOG_LEVEL_FATAL = 4


def init(config=None, log_level=LOG_LEVEL_INFO, process_name=None):
    global _var

    _var.config = Config() if config is None else config
    _var.logger = util.create_class_instance(_var.config.get_param('logger', default='lidapy.RosLogger'))
    _var.ipc = util.create_class_instance(_var.config.get_param('ipc', default='lidapy.RosCommunicationProxy'))

    if process_name:
        _var.ipc.initialize_node(process_name, log_level=log_level)

    return _var


class Config(object):
    GLOBAL_SECTION = 'global'

    def __init__(self, file_path=None, use_param_service=False):
        super(Config, self).__init__()

        # shared configuration dictionary containing key/value pairs
        # for each section in the configuration file
        #
        # format:
        # {
        #   section_1 : { param_1 : value, param_2 : value}
        #   section_2 : { param_1 : value, param_2 : value }
        # }
        self._config = {}

        # The parameter is used for runtime updates of parameter values
        self._param_service = ParameterService() if use_param_service else None

        if file_path:
            self._load(file_path)

    def _load(self, config_file):
        parser = ConfigParser.SafeConfigParser()

        if not parser.read(config_file):
            raise IOError('Failed to read configuration file: {}'.format(config_file))

        for section in parser.sections():
            self._config[section] = {}
            for name, value in parser.items(section):
                self._config[section][name] = value

                if self._param_service:
                    self._param_service.set_param(name, value, section=section)

    def set_param(self, name, value, section='global'):
        try:
            self._config[section][name] = value
        except KeyError:
            self._config[section] = {}
            self._config[section][name] = value

        if self._param_service:
            self._param_service.set_param(name, value, section)

    def get_param(self, name, section='global', default=None, func=None):
        if self._param_service:
            value = self._param_service.get_param(name, section, default)
        else:
            try:
                value = self._config[section][name]
            except KeyError:
                value = None

        if value is None:
            value = default

        if callable(func):
            value = func(value)

        return value

    def get_type_or_global_param(self, section, name, default=None):

        # Try to find param_name under param_type params
        param_value = self.get_param(name=name, section=section)
        if param_value is not None:
            return param_value

        # Try to find param_name under global params
        param_value = self.get_param(name=name)
        if param_value is not None:
            return param_value

        return default


def get_param(name, section='global', default=None, func=None):
    return _var.config.get_param(name, section, default, func)


class TaskManager(object):
    def __init__(self):
        self._tasks = collections.deque()

    def __len__(self):
        return len(self._tasks)

    def __contains__(self, item):
        if item in self._tasks:
            return True

    def __iter__(self):
        return self._tasks.__iter__()

    # A factory method that can be used with defaultdict
    def __call__(self):
        return TaskManager()

    def add(self, task):
        self._tasks.appendleft(task)

    def stop(self):
        for task in self._tasks:
            task.stop()

    def start(self):
        """ Starts each of the tasks registered with this module.

        :return: None
        """
        for task in self._tasks:
            task.start()


# A dictionary of TaskManagers
task_managers = collections.defaultdict(TaskManager)  # type: dict

# Standard Status Codes
COMPLETE = 'complete'
ERROR = 'error'
PENDING = 'pending'
RUNNING = 'running'
SHUTTING_DOWN = 'shutting_down'


class LIDARunnable(object):
    def __init__(self, name):
        super(LIDARunnable, self).__init__()

        self.name = name
        self.status = PENDING

    def is_shutting_down(self):
        return _var.ipc.is_shutting_down()

    def wait(self):
        _var.ipc.wait(self.rate_in_hz)

    @property
    def rate_in_hz(self):
        return int(_var.config.get_type_or_global_param(section=self.name, name='rate_in_hz', default=100))


class LIDAProcess(multiprocessing.Process, LIDARunnable):
    def __init__(self, name, tasks):
        multiprocessing.Process.__init__(self, name=name)
        LIDARunnable.__init__(self, name=name)

        self.tasks = [] if tasks is None else tasks  # type: # list
        self.task_manager = task_managers[name]  # type: TaskManager

    def run(self):
        loginfo('LIDAProcess [pid = {}; name = {}] beginning execution'.format(getpid(), self.name))

        try:
            self._initialize()

            while self.status is RUNNING:
                self._update_status()
                self.wait()

        except Exception as self.exception:
            self.status = ERROR
            logerror(self.exception)
            logerror(traceback.format_exc())
        finally:
            self._finalize()

        loginfo('LIDAProcess [pid = {}; name = {}; status = {}] completing execution'.format(getpid(), self.name,
                                                                                             self.status))

    def _initialize(self):
        _var.ipc.initialize_node(self.name, log_level=_var.logger.log_level)

        self.initialize()

        for task in self.tasks:
            self.task_manager.add(task)

        for task in self.task_manager:
            task.start()

        self.status = RUNNING

    def initialize(self):
        """This method can be overridden to customize initialization.

        :return: None
        """
        pass

    def _finalize(self):
        self.status = COMPLETE

        self.finalize()

    def finalize(self):
        """This method can be overridden to customize finalization.

        :return: None
        """
        pass

    def _update_status(self):
        if self.is_shutting_down():
            self.status = SHUTTING_DOWN
        else:
            for task in self.task_manager:
                if task.status is ERROR:
                    self.status = ERROR

            self.update_status()

    def update_status(self):
        """This method can be overridden to customize status updates.

        :return: None
        """
        pass


class LIDAModule(LIDAProcess):
    """ The LIDAModule class is used to sub-divide an agent into high-level processing components called modules. """

    def __init__(self, name, tasks=None):
        super(LIDAModule, self).__init__(name, tasks)


class LIDAThread(threading.Thread, LIDARunnable):
    def __init__(self, callback, name=None, callback_args=None, exec_count=-1):

        if name is None:
            name = util.generate_random_name(prefix='Thread_', length=16)

        threading.Thread.__init__(self, name=name)
        LIDARunnable.__init__(self, name=name)

        self.callback = callback
        self.callback_args = callback_args if callback_args is not None else []
        self.exec_count = exec_count
        self.exception = None

        if self.exec_count is not None:
            if type(self.exec_count) is not int:
                raise Exception('Execution count must be a positive integer if specified.')

    def run(self):
        loginfo('LIDAThread [name = {}] beginning execution'.format(self.name))

        try:
            self._initialize()

            while self.status is RUNNING:
                self.callback(*self.callback_args)
                self._update_status()
                self.wait()
        except Exception as self.exception:
            self.status = ERROR
            logerror(self.exception)
            logerror(traceback.format_exc())
        finally:
            self._finalize()

        loginfo(
            'LIDAThread [name = {}; status = {}] completing execution'.format(self.name, self.status))

    def _initialize(self):
        self.status = RUNNING

    def initialize(self):
        """This method can be overridden to customize initialization.

        :return: None
        """
        pass

    def _finalize(self):
        if self.status != ERROR:
            self.status = COMPLETE

    def finalize(self):
        """This method can be overridden to customize finalization.

        :return: None
        """
        pass

    def _update_status(self):
        # Check for termination conditions
        if self.is_shutting_down():
            self.status = COMPLETE

        elif self.exec_count is not -1:
            self.exec_count -= 1

            if self.exec_count <= 0:
                self.status = COMPLETE

    def update_status(self):
        """This method can be overridden to customize status updates.

        :return: None
        """
        pass


class Task(LIDARunnable):
    def __init__(self, name, callback, exec_count=-1):
        super(Task, self).__init__(name=name)

        self.callback = callback
        self.exec_count = exec_count

        self._thread = LIDAThread(name=self.name, callback=self.callback, exec_count=self.exec_count)
        self._thread.daemon = True

    def stop(self):
        self.status = COMPLETE

    def start(self):
        self._thread.start()

    def wait_until_complete(self):
        if threading.currentThread() is not self._thread:
            self._thread.join()


class DecayTask(Task):
    def __init__(self, name, strategy, target, getter=None, setter=None, exec_count=-1):
        super(DecayTask, self).__init__(name=name, callback=self.callback, exec_count=exec_count)

        self.strategy = strategy
        self.target = target
        self.getter = getter
        self.setter = setter

        if self.getter is None:
            self.getter = self.default_getter

        if self.setter is None:
            self.setter = self.default_setter

    def callback(self, *args, **kwargs):
        logdebug('Executing BackgroundDecayTask [name = {}]'.format(self.name))

        for x in self.target:
            current_value = self.getter(x)
            new_value = self.strategy.get_next_value(current_value, self.rate_in_hz)
            self.setter(x, new_value)

    @staticmethod
    def default_getter(activatable):
        return activatable.activation

    @staticmethod
    def default_setter(activatable, value):
        activatable.activation = value


class Activatable(object):
    DEFAULT_ACTIVATION = 0.0
    DEFAULT_BASE_LEVEL_ACTIVATION = 0.0
    DEFAULT_INCENTIVE_SALIENCE = 0.0
    DEFAULT_REMOVAL_THRESHOLD = 0.0

    def __init__(self, activation=None, base_level_activation=None, incentive_salience=None, removal_threshold=None):

        self._activation = 0.0
        self._base_level_activation = 0.0
        self._incentive_salience = 0.0
        self._removal_threshold = 0.0

        if activation is None:
            self.activation = self.DEFAULT_ACTIVATION
        else:
            self.activation = activation

        if base_level_activation is None:
            self.base_level_activation = self.DEFAULT_BASE_LEVEL_ACTIVATION
        else:
            self.base_level_activation = base_level_activation

        if incentive_salience is None:
            self.incentive_salience = self.DEFAULT_INCENTIVE_SALIENCE
        else:
            self.incentive_salience = incentive_salience

        if removal_threshold is None:
            self.removal_threshold = self.DEFAULT_REMOVAL_THRESHOLD
        else:
            self.removal_threshold = removal_threshold

    @property
    def activation(self):
        return self._activation

    @activation.setter
    def activation(self, activation):
        if activation < 0.0:
            self._activation = 0.0
        elif activation > 1.0:
            self._activation = 1.0
        else:
            self._activation = activation

    @property
    def base_level_activation(self):
        return self._base_level_activation

    @base_level_activation.setter
    def base_level_activation(self, base_level_activation):
        if base_level_activation < 0.0:
            self._base_level_activation = 0.0
        elif base_level_activation > 1.0:
            self._base_level_activation = 1.0
        else:
            self._base_level_activation = base_level_activation

    @property
    def incentive_salience(self):
        return self._incentive_salience

    @incentive_salience.setter
    def incentive_salience(self, incentive_salience):
        if incentive_salience < 0.0:
            self._incentive_salience = 0.0
        elif incentive_salience > 1.0:
            self._incentive_salience = 1.0
        else:
            self._incentive_salience = incentive_salience

    @property
    def removal_threshold(self):
        return self._removal_threshold

    @removal_threshold.setter
    def removal_threshold(self, removal_threshold):
        if removal_threshold < 0.0:
            self._removal_threshold = 0.0
        elif removal_threshold > 1.0:
            self._removal_threshold = 1.0
        else:
            self._removal_threshold = removal_threshold


class CognitiveContent(Activatable):
    def __init__(self, value):
        super(CognitiveContent, self).__init__()
        self._value = value

    def __getattr__(self, attr):
        if attr == '_value':
            raise AttributeError

        attr_v = getattr(self._value, attr)

        if callable(attr_v):
            def wrapper(*args, **kwargs):
                result = attr_v(*args, **kwargs)
                if result is self._value:
                    return self
                return result

            return wrapper
        else:
            return attr_v

    # TODO: Need a way to automatically wrap the special methods
    def __len__(self):
        return len(self._value)

    def __setitem__(self, key, value):
        if '__setitem__' in dir(self._value):
            self._value[key] = value
        else:
            raise TypeError("Indexing not supported")

    def __getitem__(self, key):
        if '__getitem__' in dir(self._value):
            return self._value[key]
        else:
            raise TypeError("Indexing not supported")


class Codelet(Task):
    def __init__(self, name, callback, base_level_activation, removal_threshold, exec_count):
        super(Codelet, self).__init__(name, callback, exec_count)

        self.base_level_activation = base_level_activation
        self.removal_threshold = removal_threshold


class Sensor(object):
    def __init__(self, name, topic=None):
        self.name = name
        self.topic = topic

    @property
    def data(self):
        return self.topic._init_subscriber.receive


class Behavior(Activatable):
    def __init__(self, unique_id, action=None, context_conditions=set(), adding_list=set(), deleting_list=set()):
        super(Behavior, self).__init__()

        self.unique_id = unique_id
        self.action = action
        self.context_conditions = set(context_conditions)
        self.adding_list = set(adding_list)
        self.deleting_list = set(deleting_list)


class Action(Activatable):
    def __init__(self, unique_id):
        super(Action, self).__init__()

        self.unique_id = unique_id


class Condition(Activatable):
    def __init__(self, unique_id, predicate):
        super(Condition, self).__init__()

        self.unique_id = unique_id
        self.predicate = predicate

    def check(self, *args, **kwargs):
        return self.predicate(*args, **kwargs)


class Topic(object):
    def __init__(self, name, msg_type=None, queue_size=2, preprocessor=None, postprocessor=None):
        super(Topic, self).__init__()

        self.name = name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor

        self._publisher = None  # type: TopicPublisher
        self._subscriber = None  # type: TopicSubscriber

        self._listener = None

    def _init_publisher(self):
        if self._publisher is None:
            loginfo('Initializing publisher for topic {}'.format(self.name))

            self._publisher \
                = _var.ipc.get_publisher(topic_name=self.name,
                                         msg_type=self.msg_type,
                                         max_queue_size=self.queue_size,
                                         preprocessor=self.preprocessor)

    def _init_subscriber(self, listener=None):
        if self._subscriber is None:
            loginfo('Initializing subscriber for topic {}'.format(self.name))

            self._subscriber = \
                _var.ipc.get_subscriber(topic_name=self.name,
                                        msg_type=self.msg_type,
                                        max_queue_size=self.queue_size,
                                        postprocessor=self.postprocessor)

            if listener:
                self._subscriber.add_listener(listener)

            return listener

    def send(self, msg):
        logdebug('[{}] Sending message: {}'.format(getpid(), msg))
        if not self._publisher:
            self._init_publisher()

        self._publisher.publish(msg)

    def receive(self, timeout=None):
        if not self._subscriber:
            self._listener = self._init_subscriber(MsgListener())

        return self._listener.receive(timeout)

    def add_listener(self):
        if not self._subscriber:
            self._init_subscriber()

        new_listener = MsgListener(self.queue_size)
        self._subscriber.add_listener(new_listener)
        return new_listener


class MsgListener(object):
    def __init__(self, queue_size=1):
        self.event = threading.Event()
        self.msg_queue = collections.deque(maxlen=queue_size)

    def receive(self, timeout=None):
        self.event.wait(timeout)
        try:
            msg = self.msg_queue.popleft()
        except IndexError:
            msg = None
            self.event.clear()

        return msg

    def notify(self, msg):
        self.msg_queue.append(copy.deepcopy(msg))
        self.event.set()


class TopicPublisher(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, topic_name, msg_type, queue_size, preprocessor):
        super(TopicPublisher, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.preprocessor = preprocessor

    @abc.abstractmethod
    def publish(self, msg):
        pass


class TopicSubscriber(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, topic_name, msg_type, queue_size, postprocessor):
        super(TopicSubscriber, self).__init__()

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.postprocessor = postprocessor

        self._listeners = collections.deque()

    def add_listener(self, listener):
        # TODO: Do I need to lock this or have a queue for register requests
        # TODO: and let a single thread in the master handle this?
        # Add thread to list of observers for this master
        try:
            self._listeners.append(listener)
        except KeyError:
            self._listeners = []
            self._listeners.append(listener)

    def remove_listener(self, listener):
        self._listeners.remove(listener)

    def notify_all(self, msg):
        for o in self._listeners:
            o.notify(msg)


# class Service(object):
#     def __init__(self, name, callback=None):
#         super(Service, self).__init__()
#
#         self.name = name
#         self.service_class = GenericService
#         self.callback = callback
#
#         self._service = None
#         self._client = None
#
#     def register(self):
#         if self._service is None:
#             loginfo('Registering new service [{}]'.format(self.name))
#             self._service = _var.ipc.get_service(self.name,
#                                                  self.service_class,
#                                                  self.callback)
#
#         return self._service
#
#     @property
#     def client(self):
#         if self._client is None:
#             self._client = _var.ipc.get_service_proxy(self.name, self.service_class)
#
#         return self._client
#
#
# class ServiceClient(object):
#     def __init__(self, service_name, service_class):
#         super(ServiceClient, self).__init__()
#
#         self.service_name = service_name
#         self.service_class = service_class
#
#     def get_service_proxy(self):
#         return _var.ipc.get_service_proxy(self.service_name, self.service_class)


class Logger(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, log_level=None):
        self.log_level = log_level

    @abc.abstractmethod
    def debug(self, msg):
        pass

    @abc.abstractmethod
    def info(self, msg):
        pass

    @abc.abstractmethod
    def warn(self, msg):
        pass

    @abc.abstractmethod
    def error(self, msg):
        pass

    @abc.abstractmethod
    def fatal(self, msg):
        pass

    @property
    def log_level(self):
        return self._log_level

    @log_level.setter
    def log_level(self, value):
        if type(value) is not int:
            raise ('Invalid log level: {}'.format(value))

        self._log_level = value


class CommunicationProxy(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, name):
        self.name = name

        self._is_shutting_down = False

    @abc.abstractmethod
    def initialize_node(self, name, log_level):
        pass

    @abc.abstractmethod
    def get_publisher(self, topic_name, msg_type, max_queue_size, preprocessor):
        # type: (...) -> TopicPublisher
        pass

    @abc.abstractmethod
    def get_subscriber(self, topic_name, msg_type, max_queue_size, postprocessor):
        pass

    @abc.abstractmethod
    def get_service(self, service_name, service_class, callback):
        pass

    @abc.abstractmethod
    def get_service_proxy(self, service_name, service_class):
        pass

    def force_shutdown(self):
        self._is_shutting_down = True

    def is_shutting_down(self):
        return self._is_shutting_down

    def wait(self, rate_in_hz):
        try:
            time.sleep(1.0 / rate_in_hz)
        except Exception:
            pass

    def get_current_time(self):
        return datetime.datetime.now()


class LocalCommunicationProxy(CommunicationProxy):
    def __init__(self):
        super(LocalCommunicationProxy, self).__init__('LocalCommunicationProxy')

        # A dictionary of queues
        # {
        #  topic_name_1 : msg_queue_1,
        #  topic_name_2 : msg_queue_2,
        # }
        self.msg_queues = {}

    def initialize_node(self, name, log_level=None):
        # No-op
        pass

    def get_publisher(self, topic_name, msg_type=None, max_queue_size=None, preprocessor=None):
        # type: (...) -> LocalTopicPublisher
        if topic_name not in self.msg_queues:
            self.msg_queues[topic_name] = LocalMessageQueue(topic_name, msg_type)

        return LocalTopicPublisher(self.msg_queues[topic_name], queue_size=max_queue_size, preprocessor=preprocessor)

    def get_subscriber(self, topic_name, msg_type=None, max_queue_size=None, postprocessor=None):
        if topic_name not in self.msg_queues:
            self.msg_queues[topic_name] = LocalMessageQueue(topic_name, msg_type)

        return LocalTopicSubscriber(msg_queue=self.msg_queues[topic_name],
                                    queue_size=max_queue_size,
                                    postprocessor=postprocessor)

    def get_service(self, service_name, service_class, callback):
        raise NotImplementedError

    def get_service_proxy(self, service_name, service_class):
        raise NotImplementedError


class RosCommunicationProxy(CommunicationProxy):
    def __init__(self):
        super(RosCommunicationProxy, self).__init__('RosCommunicationProxy')

        self.log_level_map = {LOG_LEVEL_DEBUG: rospy.DEBUG,
                              LOG_LEVEL_INFO: rospy.INFO,
                              LOG_LEVEL_WARN: rospy.WARN,
                              LOG_LEVEL_ERROR: rospy.ERROR,
                              LOG_LEVEL_FATAL: rospy.FATAL}

    def initialize_node(self, name, log_level=LOG_LEVEL_INFO):
        rospy.init_node(name, log_level=self.log_level_map[log_level])

    def get_publisher(self, topic_name, msg_type=std_msgs.String, max_queue_size=None, preprocessor=None):
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
        try:
            waiter = rospy.Rate(rate)
            waiter.sleep()
        except Exception:
            pass

    def get_current_time(self):
        return rospy.get_rostime()


class ParameterService(object):
    def get_param(self, name, section, default=None):
        fully_qualified_name = self.get_qualified_name(section, name)
        return rospy.get_param(fully_qualified_name, default)

    def set_param(self, name, value, section):
        fully_qualified_name = self.get_qualified_name(section, name)
        return rospy.set_param(fully_qualified_name, value)

    @staticmethod
    def get_qualified_name(section, name):
        return '/'.join(['lida', section, name])


class RosTopicPublisher(TopicPublisher):
    def __init__(self, topic_name, msg_type=None, queue_size=None, preprocessor=None):
        super(RosTopicPublisher, self).__init__(topic_name, msg_type, queue_size, preprocessor)

        self.msg_type = std_msgs.String if msg_type is None else msg_type
        self.queue_size = 10

        if preprocessor is None:
            if self.msg_type is std_msgs.String:
                def default_preprocessor(msg):
                    return MsgUtils.serialize(msg)

                self.preprocessor = default_preprocessor

        self._publisher = rospy.Publisher(name=self.topic_name,
                                          data_class=self.msg_type,
                                          queue_size=self.queue_size)

    def publish(self, msg):
        if callable(self.preprocessor):
            try:
                msg = self.preprocessor(msg)
            except:
                pass

        self._publisher.publish(msg)


class RosTopicSubscriber(TopicSubscriber):
    def __init__(self, topic_name, msg_type=None, queue_size=None, postprocessor=None):
        super(RosTopicSubscriber, self).__init__(topic_name=topic_name,
                                                 msg_type=std_msgs.String if msg_type is None else msg_type,
                                                 queue_size=1 if queue_size is None else queue_size,
                                                 postprocessor=postprocessor)

        if postprocessor is None:
            if self.msg_type is std_msgs.String:
                def default_postprocessor(msg):
                    return MsgUtils.deserialize(RosMsgUtils.unwrap(msg, 'data'))

                self.postprocessor = default_postprocessor

        self._subscriber = rospy.Subscriber(name=self.topic_name,
                                            data_class=self.msg_type,
                                            callback=self._subscribe,
                                            callback_args=None,
                                            queue_size=self.queue_size)

    def _subscribe(self, msg, *args):
        if callable(self.postprocessor):
            try:
                msg = self.postprocessor(msg)
            except:
                pass

        # Notify listeners
        self.notify_all(msg)


class LocalTopicPublisher(TopicPublisher):
    def __init__(self, msg_queue, queue_size=None, preprocessor=None):
        super(LocalTopicPublisher, self).__init__(topic_name=msg_queue.name,
                                                  msg_type=msg_queue.msg_type,
                                                  queue_size=queue_size,
                                                  preprocessor=preprocessor)

        self.msg_queue = msg_queue

    def publish(self, msg):
        if msg is None:
            raise Exception(
                'Attempted to publish msg on topic = {} with value = None'.format(
                    self.msg_queue.topic_name))

        if callable(self.preprocessor):
            try:
                msg = self.preprocessor(msg)
            except:
                pass

        self.msg_queue.push(msg)


class LocalTopicSubscriber(TopicSubscriber):
    def __init__(self, msg_queue, queue_size=None, postprocessor=None):
        super(LocalTopicSubscriber, self).__init__(topic_name=msg_queue.name,
                                                   msg_type=msg_queue.msg_type,
                                                   queue_size=queue_size,
                                                   postprocessor=postprocessor)

        self.msg_queue = msg_queue

        self._listener = LIDAThread(callback=self._listener_func)
        self._listener.daemon = True
        self._listener.start()

    def _listener_func(self):
        msg = self.msg_queue.pop()
        if callable(self.postprocessor):
            try:
                msg = self.postprocessor(msg)
            except:
                pass

        self.notify_all(msg)


class LocalMessageQueue(object):
    def __init__(self, name=None, msg_type=None, max_queue_size=None):
        super(LocalMessageQueue, self).__init__()

        if name is None:
            name = generate_random_name(prefix='LocalQueue_', length=16)

        self.name = name
        self.msg_type = msg_type
        self.max_queue_size = max_queue_size
        self.event = threading.Event()

        self._queue = collections.deque(maxlen=max_queue_size)

    def push(self, msg):
        if msg is not None:
            self._queue.append(msg)
            self.event.set()

    def pop(self):
        try:
            msg = self._queue.popleft()
        except IndexError:
            msg = None
            self.event.clear()

        return msg

    def peek(self):
        try:
            msg = self._queue[0]
        except IndexError:
            msg = None

        return msg

    def __len__(self):
        return len(self._queue)


class ConsoleLogger(Logger):
    MSG_FORMAT = '{timestamp} [{log_level}] {message}'

    def __init__(self, log_level=LOG_LEVEL_INFO):
        super(ConsoleLogger, self).__init__(log_level)

    def debug(self, msg):
        if LOG_LEVEL_DEBUG >= self.log_level:
            print(self._get_formatted_msg(msg, 'DEBUG'), file=sys.stdout)

    def info(self, msg):
        if LOG_LEVEL_INFO >= self.log_level:
            print(self._get_formatted_msg(msg, 'INFO'), file=sys.stdout)

    def warn(self, msg):
        if LOG_LEVEL_WARN >= self.log_level:
            print(self._get_formatted_msg(msg, 'WARN'), file=sys.stderr)

    def error(self, msg):
        if LOG_LEVEL_ERROR >= self.log_level:
            print(self._get_formatted_msg(msg, 'ERROR'), file=sys.stderr)

    def fatal(self, msg):
        if LOG_LEVEL_FATAL >= self.log_level:
            print(self._get_formatted_msg(msg, 'FATAL'), file=sys.stderr)

    def _get_formatted_msg(self, msg, level):
        return ConsoleLogger.MSG_FORMAT.format(timestamp=datetime.datetime.now(),
                                               log_level=str.upper(level),
                                               message=msg)


class RosLogger(Logger):
    def __init__(self, log_level=LOG_LEVEL_INFO):
        super(RosLogger, self).__init__(log_level)

    def debug(self, msg):
        rospy.logdebug(msg)

    def info(self, msg):
        rospy.loginfo(msg)

    def warn(self, msg):
        rospy.logwarn(msg)

    def error(self, msg):
        rospy.logerr(msg)

    def fatal(self, msg):
        rospy.logfatal(msg)


def logdebug(msg):
    _var.logger.debug(msg)


def loginfo(msg):
    _var.logger.info(msg)


def logwarn(msg):
    _var.logger.warn(msg)


def logerror(msg):
    _var.logger.error(msg)


def logfatal(msg):
    _var.logger.fatal(msg)
