import os
from multiprocessing import Process

from lidapy.util import comm, logger


class FrameworkProcess(Process):

    def __init__(self, name, **kwargs):
        super(FrameworkProcess, self).__init__(name=name)

        self.name = name

        self.log_level = kwargs.get("log_level")

        self.config = kwargs.get("config")
        if self.config is None:
            raise RuntimeError("Illegal state: agent configuration is undefined.")

        self._status = "starting"

    def run(self):

        try:
            self.initialize()

            while not self.is_complete():
                logger.debug(
                    "Process [name = {}; pid = {}] has status = \"{}\"".format(self.name, os.getpid(), self.status))

                if self.is_running():
                    self.call()

                self.wait()

        except Exception as e:
            logger.fatal(
                "Process [name = {}; pid = {}] caught exception in run method: {}".format(self.name, os.getpid(), e))
            self.status = "error"

        self.finalize()

    def get_rate(self):
        return int(self.config.get_type_or_global_param(self.name, "rate_in_hz", 100))

    def wait(self):
        comm.wait(self.get_rate())

    # May be overridden to customize initialization
    def initialize(self):
        logger.info("Process [name = {}; pid = {}] beginning execution".format(self.name, os.getpid()))

        # Register this process with communication infrastructure
        comm.initialize(self.name, log_level=self.log_level)

        self.status = "running"

    # May be overridden to customize finalization
    def finalize(self):
        logger.info("Process [name = {}; pid = {}] complete with status = {}".format(self.name, os.getpid(),
                                                                                     self.status))

        self.status = "complete"

    # Must be overridden
    def call(self):
        pass

    def is_running(self):
        return "running" == self.status

    def is_complete(self):
        return "complete" == self.status or comm.shutting_down()

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, new_status):
        self._status = new_status
