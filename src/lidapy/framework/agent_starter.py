from argparse import ArgumentParser

from lidapy.framework.agent import AgentConfig
from lidapy.module.action_selection import ActionSelection
from lidapy.module.conscious_contents_queue import ConsciousContentsQueue
from lidapy.module.current_situational_model import CurrentSituationalModel
from lidapy.module.episodic_memory import EpisodicMemory
from lidapy.module.global_workspace import GlobalWorkspace
from lidapy.module.perceptual_associative_memory import PerceptualAssociativeMemory
from lidapy.module.procedural_memory import ProceduralMemory
from lidapy.module.sensory_memory import SensoryMemory
from lidapy.module.sensory_motor_memory import SensoryMotorMemory
from lidapy.module.spatial_memory import SpatialMemory
from lidapy.module.transient_episodic_memory import TransientEpisodicMemory
from lidapy.module.workspace import Workspace


class AgentStarter(object):
    def __init__(self, **kwargs):

        self._module_dict = None
        self._args_parser = None
        self._config = None

        self._initialize_args_parser()

        self.args, self.unknown = self._args_parser.parse_known_args()

        self._initialize_module_dict(module_dict=kwargs.get("module_dict"))
        self._initialize_agent_config(config_file=kwargs.get("config_file"))

    def _initialize_args_parser(self):
        self._args_parser = ArgumentParser()
        self._args_parser.add_argument("-m", "--module_name", help="The name of the module to launch.")
        self._args_parser.add_argument("-f", "--config_file", help="The filepath to the agent configuration file.")

    def _initialize_module_dict(self, module_dict=None):

        if module_dict is None:

            modules = [
                ActionSelection,
                ConsciousContentsQueue,
                CurrentSituationalModel,
                EpisodicMemory,
                GlobalWorkspace,
                PerceptualAssociativeMemory,
                ProceduralMemory,
                SensoryMemory,
                SensoryMotorMemory,
                SpatialMemory,
                TransientEpisodicMemory,
                Workspace,
            ]

            self._module_dict = {module.get_module_name(): module for module in modules}
        else:
            self._module_dict = module_dict

    def _initialize_agent_config(self, config_file):
        if self._config is None:
            cf = config_file or self.args.config_file
            self._config = AgentConfig(config_file=cf)

        return self._config

    def add_module(self, module):
        self._module_dict[module.get_module_name()] = module

    def remove_module(self, module):
        self._module_dict.pop(module.name)

    def start(self, **kwargs):

        module_name = kwargs.get("module_name") or self.args.module_name
        module_class = self._module_dict.get(module_name)

        if module_class is not None:
            print "Starting module {}".format(self.args.module_name)
            module_obj = module_class(config=self._config, cmdline_args=self.args, **kwargs)
            module_obj.run()
        else:
            print "Unknown module: {}".format(self.args.module_name)
