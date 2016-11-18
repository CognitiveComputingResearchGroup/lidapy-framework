from ConfigParser import SafeConfigParser
from argparse import ArgumentParser

from lidapy.framework.shared import FrameworkDependencyService, FrameworkObject
from lidapy.util.comm import ParameterService
from lidapy.util.comm import RosCommunicationProxy
from lidapy.util.logger import RosLogger


class AgentConfig(FrameworkObject):
    # Default filepath for agent configuration file
    default_agent_config_filepath = "configs/agent.conf"

    # shared configuration dictionary containing key/value pairs
    # for each section in the configuration file
    #
    # format:
    # {
    #   section_1 : { key_1 : value_1, key_2 : value_2 }
    #   section_2 : { key_1 : value_1, key_2 : value_2 }
    # }
    _config = {}

    # a reference to a parameter service (if applicable)
    _param_service = None

    _initialized = False

    def __init__(self, config_file=None, config_file_override=False):
        super(AgentConfig, self).__init__()

        if not AgentConfig._initialized:
            if config_file_override is False:
                if config_file is None:
                    raise ValueError("Configuration file must be supplied")
                else:
                    found_filepath = self._find_config_file(config_file)
                    if not found_filepath:
                        raise IOError("Failed to find usable agent configuration file")

                    self._load_config(found_filepath)
            else:
                self.logger.warn("Agent configuration file overridden by user request")

            if self._using_param_service():
                self._load_param_service()

            AgentConfig._initialized = True

    def _find_config_file(self, config_filepath):

        candidate_filepaths \
            = [config_filepath,
               AgentConfig.default_agent_config_filepath]

        for filepath in candidate_filepaths:
            if self._found_config_file_at(filepath):
                self.logger.info("Using agent configuration at {}".format(filepath))
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

    def _load_file_config(self, config_file):
        self.logger.info("Loading parameters from configuration file [{}]".format(config_file))

        parser = SafeConfigParser()
        parser.read(config_file)

        for section in parser.sections():
            self.logger.info("Loading parameters in section [{}]".format(section))
            AgentConfig._config[section] = {}
            for key, value in parser.items(section):
                self.logger.info("Loading parameter [{} = {}]".format(key, value))
                AgentConfig._config[section][key] = value

    def _load_param_service(self):
        self.logger.info("Initializing parameter service")
        AgentConfig._param_service = ParameterService()

        for param_type in AgentConfig._config:
            for param_name, param_value in AgentConfig._config[param_type].items():
                AgentConfig._param_service.set_param(param_type, param_name, param_value)

    def _using_param_service(self):
        param_value = self.get_param("global_params", "use_param_service", "False")

        if param_value.lower() in ["true", "1"]:
            return True
        else:
            return False

    def set_global_param(self, param_name, value):
        self.set_param("global_params", param_name, value)

    def get_global_param(self, param_name, default_value=None):
        param_value = self.get_param("global_params", param_name, default_value)
        return param_value

    def set_param(self, param_type, param_name, value):
        if not AgentConfig._config.has_key(param_type):
            AgentConfig._config[param_type] = {}

        AgentConfig._config[param_type][param_name] = value

    def get_param(self, param_type, param_name, default_value=None):
        if AgentConfig._param_service is None:
            try:
                param_value = AgentConfig._config[param_type][param_name]
            except KeyError:
                self.logger.debug("Parameter [section: {}][key: {}] does not exist.".format(param_type, param_name))
                param_value = None

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


class AgentStarter(FrameworkObject):
    def __init__(self, **kwargs):
        super(AgentStarter, self).__init__()

        self._module_dict = {}
        self._args_parser = None
        self._config = None

        self._fds = FrameworkDependencyService()

        # Add logger and communication proxy to dependencyService
        self._fds["logger"] = RosLogger()
        self._fds["ipc_proxy"] = RosCommunicationProxy()

        self._initialize_args_parser()

        self._args, self._unknown = self._args_parser.parse_known_args()

        self._config = self._initialize_agent_config(config_file=kwargs.get("config_file"))
        self._fds["config"] = self._config

    def _initialize_args_parser(self):
        self._args_parser = ArgumentParser()
        self._args_parser.add_argument("-m", "--module_name", help="The name of the module to launch.")
        self._args_parser.add_argument("-f", "--config_file", help="The filepath to the agent configuration file.")

    def _initialize_agent_config(self, config_file):
        cf = config_file or self._args.config_file
        config = AgentConfig(cf)

        return config

    def _initialize_dependency_service(self):
        fds = FrameworkDependencyService()

        fds["config"] = self._config

    def add_module(self, module):
        module_name = module.get_module_name()
        self._module_dict[module_name] = module

    def start(self, **kwargs):

        module_name = kwargs.get("module_name") or self._args.module_name
        module_class = self._module_dict.get(module_name)

        if module_class is not None:
            self.logger.info("Starting module {}".format(module_name))
            module_obj = module_class(**kwargs)
            module_obj.run()

        else:
            self.logger.fatal("Unknown module: {}".format(module_name))
