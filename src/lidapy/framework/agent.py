from ConfigParser import SafeConfigParser
from lidapy.util import logger
from lidapy.util.comm import ParameterService
from os import getenv


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

    def __init__(self, config_filepath):
        if not AgentConfig._initialized:
            found_filepath = self._find_config_file(config_filepath)
            if not found_filepath:
                raise IOError("Failed to find usable agent configuration file")

            self._load_config(found_filepath)

    def _find_config_file(self, config_filepath):

        candidate_filepaths = [config_filepath,
                               getenv(AgentConfig.agent_config_env_var),
                               AgentConfig.default_agent_config_filepath]

        for filepath in candidate_filepaths:
            if AgentConfig._found_config_file_at(filepath):
                return filepath

        return None

    def _found_config_file_at(self, config_filepath):
        found = False
        try:
            # Test if file exists and has read permissions
            with open(config_filepath) as file:
                found = True
        except IOError as e:
            logger.warn("Failed to find config file at filepath = {}".format(config_filepath))

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

    def get_param(self, param_type, param_name, default_value):
        param_value = default_value
        if AgentConfig._param_service is None:
            logger.debug("Retrieving parameter [type = {}, name = {}] from file config".format(param_type, param_name,
                                                                                               default_value))

            print AgentConfig._file_config
            param_value = AgentConfig._file_config[param_type][param_name]
        else:
            logger.debug(
                "Retrieving parameter [type = {}, name = {}] from parameter service".format(param_type, param_name,
                                                                                            default_value))

            param_value = AgentConfig._param_service.get_param(param_type,
                                                               param_name,
                                                               default_value)

        if param_value is None:
            logger.warn("Parameter \"{}\"not found.  Using default = {}".format(param_name, default_value))

            param_value = default_value
        else:
            logger.debug("{} = {} [default: {}]".format(param_name, param_value, default_value))

        return param_value
