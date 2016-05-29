import os
from multiprocessing import Process

from lidapy.framework.agent import AgentConfig
from lidapy.util import comm, logger


class FrameworkProcess(Process):
    def __init__(self, process_name, **kwargs):
        super(FrameworkProcess, self).__init__(process_name, **kwargs)

        self.process_name = process_name
        self.log_level = kwargs.get("log_level")

    def run(self):
        logger.info("Beginning execution for process = {} [pid = {}]".format(self.process_name, os.getpid()))

        self.initialize()

        try:

            while not comm.shutting_down():
                rate_in_hz = int(AgentConfig().get_type_or_global_param(self.process_name, "rate_in_hz", 100))
                self.advance()
                comm.wait(rate_in_hz)

        except Exception as e:
            logger.fatal("Caught exception in run method: {}".format(e))

    def initialize(self):

        # Register this process with communication infrastructure
        comm.initialize(self.process_name, log_level=self.log_level)

    # Must be overridden
    def advance(self):
        logger.debug("Inside advance")
