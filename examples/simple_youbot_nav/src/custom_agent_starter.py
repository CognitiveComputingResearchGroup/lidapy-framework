#!/usr/bin/env python

from custom_modules import BasicSensoryMemory, BasicSensoryMotorMemory
from lidapy.framework.agent_starter import AgentStarter

if __name__ == '__main__':

    try:
        starter = AgentStarter()

        starter.add_module(BasicSensoryMemory)
        starter.add_module(BasicSensoryMotorMemory)

        starter.start()

    except Exception as e:
        print "Received an exception: {}".format(e)

    finally:
        pass
