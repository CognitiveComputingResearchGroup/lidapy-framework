#!/usr/bin/env python

from custom_modules import BasicSensoryMemory, BasicSensoryMotorMemory, BasicPerceptualAssociativeMemory, \
    BasicWorkspace, BasicGlobalWorkspace, BasicProceduralMemory, BasicActionSelection
from lidapy.framework.agent_starter import AgentStarter

if __name__ == '__main__':

    try:
        starter = AgentStarter()

        starter.add_module("SensoryMemory", BasicSensoryMemory)
        starter.add_module("SensoryMotorMemory", BasicSensoryMotorMemory)
        starter.add_module("PerceptualAssociativeMemory", BasicPerceptualAssociativeMemory)
        starter.add_module("Workspace", BasicWorkspace)
        starter.add_module("GlobalWorkspace", BasicGlobalWorkspace)
        starter.add_module("ProceduralMemory", BasicProceduralMemory)
        starter.add_module("ActionSelection", BasicActionSelection)

        starter.start()

    except Exception as e:
        print "Received an exception: {}".format(e)

    finally:
        pass
