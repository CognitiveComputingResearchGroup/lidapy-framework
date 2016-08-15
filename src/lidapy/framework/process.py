import os
from multiprocessing import Process
from threading import Thread

from lidapy.util import comm, logger

# Status codes
COMPLETE = "complete"
ERROR = "error"
RUNNING = "running"
STARTING = "starting"


class FrameworkRunnable(object):
    def __init__(self):
        self.status = STARTING

    def is_running(self):
        return RUNNING == self.status

    def is_complete(self):
        return COMPLETE == self.status or comm.shutting_down()

    def wait(self, rate_in_hz):
        comm.wait(rate_in_hz)


class FrameworkProcess(Process, FrameworkRunnable):
    def __init__(self, name, **kwargs):
        super(FrameworkProcess, self).__init__(name=name)

        self.name = name

        self.log_level = kwargs.get("log_level")

        self.config = kwargs.get("config")
        if self.config is None:
            raise RuntimeError("Illegal state: agent configuration is undefined.")

    def run(self):

        try:
            self.initialize()

            while not self.is_complete():
                logger.debug(
                    "Process [name = {}; pid = {}] has status = \"{}\"".format(self.name, os.getpid(), self.status))

                if self.is_running():
                    self.call()

                rate_in_hz = self.get_rate()
                self.wait(rate_in_hz)

        except Exception as e:
            logger.fatal(
                "Process [name = {}; pid = {}] caught exception in run method: {}".format(self.name, os.getpid(), e))

            self.status = ERROR

        self.finalize()

    def get_rate(self):
        return int(self.config.get_type_or_global_param(self.name, "rate_in_hz", 100))

    # May be overridden to customize initialization
    def initialize(self):
        logger.info("Process [name = {}; pid = {}] beginning execution".format(self.name, os.getpid()))

        # Register this process with communication infrastructure
        comm.initialize(self.name, log_level=self.log_level)

        self.status = RUNNING

    # May be overridden to customize finalization
    def finalize(self):
        logger.info("Process [name = {}; pid = {}] complete with status = {}".format(self.name, os.getpid(),
                                                                                     self.status))

        if self.status != ERROR:
            self.status = COMPLETE

    # Must be overridden
    def call(self):
        pass


class FrameworkTask(Thread, FrameworkRunnable):
    def __init__(self, name=None, callback=None, rate_in_hz=None, args=None):
        Thread.__init__(self, name=name)
        FrameworkRunnable.__init__(self)

        self.callback = callback
        self.rate_in_hz = rate_in_hz
        self.args = args

    def run(self):

        try:
            self.status = RUNNING

            while not self.is_complete():
                if self.is_running():
                    self.callback(self.args)

                self.wait(self.rate_in_hz)

        except Exception as e:
            logger.fatal(
                "Task [name = {}; pid = {}] caught exception: {}".format(self.name, os.getpid(), e))
            self.status = ERROR

        if self.status != ERROR:
            self.status = COMPLETE
