from abc import ABCMeta, abstractmethod
from multiprocessing import Process
from threading import Thread, currentThread

from lidapy.framework.shared import FrameworkObject
from lidapy.util.functions import generate_random_name


class FrameworkRunnable(FrameworkObject):
    # Standard Status Codes
    COMPLETE = "complete"
    ERROR = "error"
    INITIALIZING = "initializing"
    PENDING = "pending"
    RUNNING = "running"

    def __init__(self, name):
        super(FrameworkRunnable, self).__init__()

        self.name = name
        self.status = self.PENDING

    def is_complete(self):
        return self.COMPLETE == self.status

    def is_shutting_down(self):
        return self.ipc_proxy.is_shutting_down()

    def wait(self):
        self.ipc_proxy.wait(self.rate_in_hz)

    @property
    def rate_in_hz(self):
        return int(self.config.get_type_or_global_param(self.name, "rate_in_hz", 100))


class FrameworkProcess(Process, FrameworkRunnable):
    __metaclass__ = ABCMeta

    def __init__(self, name):
        Process.__init__(self, name=name)
        FrameworkRunnable.__init__(self, name=name)

    def run(self):
        self.logger.info("FrameworkProcess [name = {}] beginning execution".format(self.name))
        self.initialize()

        while not self.is_complete():
            self.update_status()
            self.wait()

        self.finalize()
        self.logger.info(
            "FrameworkProcess [name = {}; status = {}] completing execution".format(self.name, self.status))

    def initialize(self):
        """

        :return:
        """
        self.ipc_proxy.initialize_node(self.name, log_level=self.logger.log_level)

    def finalize(self):
        """

        :return:
        """
        if self.status != self.ERROR:
            self.status = self.COMPLETE

    @abstractmethod
    def update_status(self):
        """

        :return:
        """
        if self.is_shutting_down():
            self.status = self.COMPLETE


class FrameworkThread(Thread, FrameworkRunnable):
    def __init__(self, callback, name=None, callback_args=None, exec_count=-1):

        if name is None:
            name = generate_random_name(prefix="Thread_", length=16)

        Thread.__init__(self, name=name)
        FrameworkRunnable.__init__(self, name=name)

        self.callback = callback
        self.callback_args = callback_args if callback_args is not None else []
        self.exec_count = exec_count
        self.exception = None

        if self.exec_count is not None:
            if type(self.exec_count) is not int:
                raise Exception("Execution count must be a positive integer if specified.")

    def run(self):
        self.logger.info("FrameworkThread [name = {}] beginning execution".format(self.name))
        self.initialize()

        try:
            while not self.is_complete():
                self.callback(*self.callback_args)
                self.update_status()
                self.wait()
        except Exception as self.exception:
            self.logger.error(
                "FrameworkThread [name = {}] received an exception in run() method: {}".format(self.name,
                                                                                               self.exception))
            self.status = self.ERROR

        self.finalize()
        self.logger.info(
            "FrameworkThread [name = {}; status = {}] completing execution".format(self.name, self.status))

    def initialize(self):
        self.status = self.RUNNING

    def finalize(self):
        if self.status != self.ERROR:
            self.status = self.COMPLETE

    def update_status(self):

        # Check for termination conditions
        if self.is_shutting_down():
            self.status = self.COMPLETE

        elif self.exec_count is not -1:
            self.exec_count -= 1

            if self.exec_count <= 0:
                self.status = self.COMPLETE


class FrameworkBackgroundTask(FrameworkRunnable):
    def __init__(self, name, callback, exec_count=-1):
        super(FrameworkBackgroundTask, self).__init__(name=name)

        self.callback = callback
        self.exec_count = exec_count

        self._thread = FrameworkThread(name=self.name, callback=self.callback, exec_count=self.exec_count)
        self._thread.daemon = True

    def start(self):
        self._thread.start()

    def wait_until_complete(self):
        if currentThread() is not self._thread:
            self._thread.join()


class BackgroundDecayTask(FrameworkBackgroundTask):
    def __init__(self, name, strategy, target, getter=None, setter=None, exec_count=-1):
        super(BackgroundDecayTask, self).__init__(name=name, callback=self.callback, exec_count=exec_count)

        self.strategy = strategy
        self.target = target

        if self.name is None:
            raise ValueError("BackgroundDecayTask: name must be provided to initializer")

        if self.strategy is None:
            raise ValueError("BackgroundDecayTask: strategy must be provided to initializer")

        if self.target is None:
            raise ValueError("BackgroundDecayTask: target must be provided to initializer")

        self.getter = getter
        self.setter = setter

        if self.getter is None:
            self.getter = self.default_getter

        if self.setter is None:
            self.setter = self.default_setter

    def callback(self, *args, **kwargs):
        self.logger.debug("Executing BackgroundDecayTask [name = {}]".format(self.name))

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
