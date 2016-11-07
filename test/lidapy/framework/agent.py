import textwrap
import unittest
from tempfile import NamedTemporaryFile

from lidapy.framework.agent import AgentConfig
from lidapy.util.logger import ConsoleLogger

# A test version of the content that could appear in
# an agent.conf file.
test_config = \
    """
    [global_params]
    rate_in_hz = 5
    use_param_service = False

    [module_1]
    parameter_1 = value1
    parameter_2 = value2

    [module_2]
    parameter_1 = value1
    parameter_2 = value2
    """


class AgentConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):

        # Use the TemporaryFile context manager for easy clean-up
        with NamedTemporaryFile() as tmp:
            cleansed_test_config = textwrap.dedent(test_config).lstrip()
            tmp.write(cleansed_test_config)
            tmp.flush()

            try:
                config = AgentConfig(config_file=tmp.name, logger=ConsoleLogger())

                # Verify that global_param value from config matches
                # expected value
                expVal = "5"
                actVal = config.get_global_param("rate_in_hz")

                assert (expVal == actVal)

                # Verify that global_param value from config matches
                # expected value
                expVal = "False"
                actVal = config.get_global_param("use_param_service")

                assert (expVal == actVal)

                # Verify that none is returned when a global parameter
                # is requested that does not exist
                expVal = None
                actVal = config.get_global_param("does_not_exist")

                assert (expVal == actVal)

                # Verify that parameter_1 value from module_1
                # matches the expected value
                expVal = "value1"
                actVal = config.get_param("module_1", "parameter_1")

                assert (expVal == actVal)

                # Verify that parameter_2 value from module_1
                # matches the expected value
                expVal = "value2"
                actVal = config.get_param("module_1", "parameter_2")

                assert (expVal == actVal)

                # Verify that parameter_1 value from module_2
                # matches the expected value
                expVal = "value1"
                actVal = config.get_param("module_2", "parameter_1")

                assert (expVal == actVal)

                # Verify that parameter_2 value from module_2
                # matches the expected value
                expVal = "value2"
                actVal = config.get_param("module_2", "parameter_2")

                assert (expVal == actVal)

                # Verify that none is returned when a parameter
                # is requested for a non-existent type
                expVal = None
                actVal = config.get_param("does_not_exist", "parameter_1")

                assert (expVal == actVal)

            except Exception as e:
                self.fail("Unexpected Exception: {}".format(e))
