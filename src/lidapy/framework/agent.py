from ConfigParser import SafeConfigParser

from lidapy.framework.shared import FrameworkDependencyService, FrameworkObject
from lidapy.util.comm import ParameterService
from lidapy.util.functions import create_class_instance


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

    def __init__(self, config_file=None, config_file_override=False, use_param_service=False):
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

            if self._using_param_service(use_param_service):
                self._load_param_service()

            AgentConfig._initialized = True

    def _find_config_file(self, config_filepath):

        candidate_filepaths \
            = [config_filepath,
               AgentConfig.default_agent_config_filepath]

        for filepath in candidate_filepaths:
            if self._found_config_file_at(filepath):
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
        parser = SafeConfigParser()
        parser.read(config_file)

        for section in parser.sections():
            AgentConfig._config[section] = {}
            for key, value in parser.items(section):
                AgentConfig._config[section][key] = value

    def _load_param_service(self):
        AgentConfig._param_service = ParameterService()

        for param_type in AgentConfig._config:
            for param_name, param_value in AgentConfig._config[param_type].items():
                AgentConfig._param_service.set_param(param_type, param_name, param_value)

    def _using_param_service(self, use_param_service):
        param_value = self.get_param("global_params", "use_param_service", str(use_param_service))

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


class AgentStarter(object):
    def __init__(self, config=None):
        super(AgentStarter, self).__init__()

        self.config = config if config is not None else AgentConfig(config_file_override=True)

        self.logger_type = self.config.get_global_param("logger", "lidapy.util.logger.ConsoleLogger")
        self.ipc_proxy_type = self.config.get_global_param("ipc_proxy", "lidapy.util.comm.LocalCommunicationProxy")

        self.logger = create_class_instance(self.logger_type)
        self.ipc_proxy = create_class_instance(self.ipc_proxy_type)

        self._fds = FrameworkDependencyService()
        self._fds["logger"] = self.logger
        self._fds["ipc_proxy"] = self.ipc_proxy
        self._fds["config"] = self.config

        self._module_dict = {}

    def add_module(self, module):
        module_name = module.get_module_name()
        self._module_dict[module_name] = module

    def start(self, module_name):
        module_class = self._module_dict.get(module_name)

        if module_class is not None:
            self.logger.info("Starting module {}".format(module_name))
            module_obj = module_class()
            module_obj.start()
        else:
            self.logger.fatal("Unknown module: {}".format(module_name))
