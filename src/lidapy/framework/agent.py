from ConfigParser import SafeConfigParser
from lidapy.util import logger
from lidapy.util.comm import ParameterService


class AgentConfig(object):

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

    def __init__(self, config_file="configs/agent.conf", use_param_service = False):
        if not AgentConfig._initialized:
            self._load_config(config_file, use_param_service)

    def _load_config(self, config_file, use_param_service = False):
        self._load_file_config(config_file)
        if use_param_service:
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
        AgentConfig._param_service = ParameterService()

        for param_type in AgentConfig._file_config:
            for param_name, param_value in AgentConfig._file_config[param_type].iter_items():
                AgentConfig._param_service.set_param(param_type, param_name, param_value)

    def get_param(self, param_type, param_name, default_value):
        param_value = default_value
        if AgentConfig._param_service is not None:
            param_value = AgentConfig._param_service.get_param(param_type,
                                                               param_name,
                                                               default_value)
        else:
            param_value = AgentConfig._file_config[param_type][param_name]

        if param_value is None:
            param_value = default_value

        logger.debug("{} = {} [default: {}]".format(param_name, param_value, default_value))

        return param_value

