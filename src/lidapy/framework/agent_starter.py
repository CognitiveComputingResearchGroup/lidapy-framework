from argparse import ArgumentParser

from lidapy.module.action_selection import ActionSelection
from lidapy.module.conscious_contents_queue import ConsciousContentsQueue
from lidapy.module.global_workspace import GlobalWorkspace
from lidapy.module.perceptual_associative_memory import PerceptualAssociativeMemory
from lidapy.module.procedural_memory import ProceduralMemory
from lidapy.module.sensory_memory import SensoryMemory
from lidapy.module.sensory_motor_memory import SensoryMotorMemory
from lidapy.module.spatial_memory import SpatialMemory
from lidapy.module.transient_episodic_memory import TransientEpisodicMemory
from lidapy.module.workspace import Workspace


class AgentStarter(object):
    def __init__(self, module_dict=None):
        self._configure_module_dict(module_dict)
        self._configure_args_parser()

    def _configure_module_dict(self, module_dict):
        self.module_dict = {
            "ActionSelection": ActionSelection,
            "ConsciousContentsQueue": ConsciousContentsQueue,
            "GlobalWorkspace": GlobalWorkspace,
            "PerceptualAssociativeMemory": PerceptualAssociativeMemory,
            "ProceduralMemory": ProceduralMemory,
            "SensoryMemory": SensoryMemory,
            "SensoryMotorMemory": SensoryMotorMemory,
            "SpatialMemory": SpatialMemory,
            "TransientEpisodicMemory": TransientEpisodicMemory,
            "Workspace": Workspace,
        }

    def _configure_args_parser(self):
        self._args_parser = ArgumentParser()
        self._args_parser.add_argument("-m", "--module_name", help="The name of the module to launch.")

    def add_module(self, module_name, module_class):
        self.module_dict[module_name] = module_class

    def remove_module(self, module_name):
        self.module_dict.pop(module_name)

    def start(self, **kwargs):
        args, unknown = self._args_parser.parse_known_args()

        module_class = self.module_dict.get(args.module_name, None)
        if module_class is not None:
            print "Starting module {}".format(args.module_name)
            module_obj = module_class(**kwargs)
            module_obj.run()
        else:
            print "Unknown module: {}".format(args.module_name)
