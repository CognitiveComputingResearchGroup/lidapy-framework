#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
import os
from multiprocessing import Process

from lidapy.framework.agent import AgentConfig
from lidapy.util import comm, logger


class FrameworkProcess(Process):
    def __init__(self, name):
        super(Process, self).__init__(name=name)

        # Register this process with communication infrastructure
        comm.initialize(name)

    def run(self):
        logger.info("Beginning execution for process = {} [pid = {}]".format(self.name, os.getpid()))
        while not comm.shutting_down():
            rate_in_hz = int(AgentConfig().get_global_param("rate_in_hz", 100))
            self.advance()
            comm.wait(rate_in_hz)

    def advance(self):
        pass
