import os
from abc import ABCMeta, abstractmethod
from multiprocessing import Process
from threading import Thread, currentThread
from traceback import format_exc

from lidapy.framework.shared import FrameworkObject

COMPLETE = "complete"
ERROR = "error"
RUNNING = "running"
STARTING = "starting"


class FrameworkRunnable(FrameworkObject):
    def __init__(self, name, **kwargs):
        super(FrameworkRunnable, self).__init__()

        self.name = name
        self.status = STARTING

    def is_running(self):
        return RUNNING == self.status

    def is_complete(self):
        return COMPLETE == self.status or self.ipc_proxy.is_shutting_down()

    def wait(self):
        self.ipc_proxy.wait(self.rate_in_hz)

    @property
    def rate_in_hz(self):
        return int(self.config.get_type_or_global_param(self.name, "rate_in_hz", 100))


class FrameworkProcess(Process, FrameworkRunnable):
    __metaclass__ = ABCMeta

    def __init__(self, name, **kwargs):
        Process.__init__(self, name=name)
        FrameworkRunnable.__init__(self, name=name, **kwargs)

    def run(self):

        try:
            self.initialize()

            while not self.is_complete():
                self.logger.debug(
                    "Process [name = {}; pid = {}] has status = \"{}\"".format(self.name, os.getpid(), self.status))

                if self.is_running():
                    self.call()

                self.wait()

        except Exception as e:

            self.logger.fatal(
                "Process [name = {}; pid = {}] caught exception in run method: {}".format(self.name, os.getpid(), e))
            self.logger.fatal("Stacktrace: {}".format(format_exc()))

            self.status = ERROR

        self.finalize()

    # May be overridden to customize initialization
    def initialize(self):
        self.logger.info("Process [name = {}; pid = {}] beginning execution".format(self.name, os.getpid()))

        # Register this process with communication infrastructure
        self.ipc_proxy.initialize_node(self.name, log_level=self.logger.log_level)

        self.status = RUNNING

    # May be overridden to customize finalization
    def finalize(self):
        self.logger.info("Process [name = {}; pid = {}] complete with status = {}".format(self.name, os.getpid(),
                                                                                          self.status))

        if self.status != ERROR:
            self.status = COMPLETE

    # Must be overridden
    @abstractmethod
    def call(self):
        pass


class FrameworkThread(Thread, FrameworkRunnable):
    def __init__(self, name, callback=None, callback_args=None, exec_count=None):
        Thread.__init__(self, name=name)
        FrameworkRunnable.__init__(self, name=name)

        self.callback = callback
        self.callback_args = callback_args
        self.exec_count = exec_count

        if self.exec_count is not None:
            if type(self.exec_count) is not int:
                raise Exception("Execution count must be a positive integer if specified.")

    def run(self):

        try:
            self.status = RUNNING

            while not self.is_complete():
                if self.is_running():
                    self.callback(self.callback_args)

                    # Check for termination conditions
                    if self.exec_count is not None:
                        self.exec_count -= 1

                        if self.exec_count <= 0:
                            self.status = COMPLETE

                self.wait()

        except Exception as e:
            self.logger.fatal(
                "Task [name = {}; pid = {}] caught exception: {}".format(self.name, os.getpid(), e))
            self.logger.fatal("Stacktrace: {}".format(format_exc()))
            self.status = ERROR

        if self.status != ERROR:
            self.status = COMPLETE


class FrameworkBackgroundTask(FrameworkRunnable):
    def __init__(self, name, callback, exec_count=-1):
        super(FrameworkBackgroundTask, self).__init__(name=name)

        self.callback = callback

        self._thread = FrameworkThread(self.name, callback=self.callback, exec_count=exec_count)

        # Indicates that this thread is a daemon thread.  A python program will exit
        # if the only remaining threads are daemon threads.
        self._thread.daemon = True

    def start(self):
        self.logger.info("FrameworkBackgroundTask [name = {}] beginning execution".format(self.name))
        self._thread.start()

    def wait_until_complete(self):
        if currentThread() != self._thread:
            self._thread.join()


class BackgroundDecayTask(FrameworkBackgroundTask):
    def __init__(self, name, strategy, target, exec_count=-1):
        super(BackgroundDecayTask, self).__init__(name=name, callback=self.callback, exec_count=exec_count)

        self.strategy = strategy
        self.target = target

    def callback(self, *args, **kwargs):
        for x in self.target:
            self.strategy.apply(x, self.rate_in_hz)
