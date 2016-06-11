from ConfigParser import SafeConfigParser
from argparse import ArgumentParser
from os import getenv

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
from lidapy.util import logger
from lidapy.util.comm import ParameterService


class AgentConfig(object):

    # Environment variable for agent configuration file
    agent_config_env_var = "LIDAPY_AGENT_CONFIG"

    # Default filepath for agent configuration file if not set in environment
    default_agent_config_filepath = "configs/agent.conf"

    # shared configuration dictionary containing key/value pairs
    # for each section in the configuration file
    #
    # format:
    # {
    #   section_1 : { key_1 : value_1, key_2 : value_2 }
    #   section_2 : { key_1 : value_1, key_2 : value_2 }
    # }
    _file_config = {}

    # a reference to a parameter service (if applicable)
    _param_service = None

    _initialized = False

    def __init__(self, config_filepath=None):
        super(AgentConfig, self).__init__()

        if not AgentConfig._initialized:
            found_filepath = self._find_config_file(config_filepath)
            if not found_filepath:
                raise IOError("Failed to find usable agent configuration file")

            self._load_config(found_filepath)
            AgentConfig._initialized = True

    def _find_config_file(self, config_filepath):

        candidate_filepaths \
            = [config_filepath,
               getenv(AgentConfig.agent_config_env_var),
               AgentConfig.default_agent_config_filepath]

        for filepath in candidate_filepaths:
            if self._found_config_file_at(filepath):
                logger.info("Using agent configuration at {}".format(filepath))
                return filepath

        return None

    def _found_config_file_at(self, config_filepath):
        if not config_filepath:
            return False

        found = False
        try:
            # Test if file exists and has read permissions
            with open(config_filepath) as file:
                found = True
        except IOError as e:
            pass

        return found

    def _load_config(self, config_file):
        self._load_file_config(config_file)
        if self._using_param_service():
            self._load_param_service()

    def _load_file_config(self, config_file):
        logger.info("Loading parameters from configuration file [{}]".format(config_file))

        parser = SafeConfigParser()
        parser.read(config_file)

        for section in parser.sections():
            logger.info("Loading parameters in section [{}]".format(section))
            AgentConfig._file_config[section] = {}
            for key, value in parser.items(section):
                logger.info("Loading parameter [{} = {}]".format(key, value))
                AgentConfig._file_config[section][key] = value

    def _load_param_service(self):
        logger.info("Initializing parameter service")
        AgentConfig._param_service = ParameterService()

        for param_type in AgentConfig._file_config:
            for param_name, param_value in AgentConfig._file_config[param_type].items():
                AgentConfig._param_service.set_param(param_type, param_name, param_value)

    def _using_param_service(self):
        param_value = self.get_param("global_params", "use_param_service", "False")

        if param_value.lower() in ["true", "1"]:
            return True
        else:
            return False

    def get_global_param(self, param_name, default_value=None):
        param_value = self.get_param("global_params", param_name, default_value)
        return param_value

    def get_param(self, param_type, param_name, default_value=None):
        if AgentConfig._param_service is None:
            param_value = AgentConfig._file_config[param_type][param_name]
        else:
            param_value = AgentConfig._param_service.get_param(param_type,
                                                               param_name,
                                                               default_value)

        if param_value is None:
            param_value = default_value

        return param_value

    def get_type_or_global_param(self, param_type, param_name, default_value=None):

        # Try to find param_name under param_type params
        param_value = self.get_param(param_type, param_name)
        if param_value is not None:
            return param_value

        # Try to find param_name under global params
        param_value = self.get_global_param(param_name)
        if param_value is not None:
            return param_value

        return default_value


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
        args = self._args_parser.parse_args()

        module_class = self.module_dict.get(args.module_name, None)
        if module_class is not None:
            print "Starting module {}".format(args.module_name)
            module_obj = module_class(**kwargs)
            module_obj.run()
        else:
            print "Unknown module: {}".format(args.module_name)
