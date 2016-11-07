#!/usr/bin/env python

from traceback import print_exc

from lidapy.framework.agent import AgentStarter

from custom_modules import BasicSensoryMemory, BasicSensoryMotorMemory

if __name__ == '__main__':

    try:
        starter = AgentStarter()

        starter.add_module(BasicSensoryMemory)
        starter.add_module(BasicSensoryMotorMemory)

        starter.start()

    except Exception as e:
        print_exc()
        print("Received an exception: {}".format(e))

    finally:
        pass
