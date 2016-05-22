import os
from multiprocessing import Process

from lidapy.framework.agent import AgentConfig
from lidapy.util import comm, logger


class FrameworkProcess(Process):
    def __init__(self, name):
        super(FrameworkProcess, self).__init__(name=name)

        self.name = name

    def run(self):
        logger.info("Beginning execution for process = {} [pid = {}]".format(self.name, os.getpid()))

        # Register this process with communication infrastructure
        comm.initialize(self.name)

        try:

            while not comm.shutting_down():
                rate_in_hz = int(AgentConfig().get_type_or_global_param(self.name, "rate_in_hz", 100))
                self.advance()
                comm.wait(rate_in_hz)

        except Exception as e:
            logger.fatal("Caught exception in run method: {}".format(e))

    # Must be overridden
    def advance(self):
        logger.debug("Inside advance")
