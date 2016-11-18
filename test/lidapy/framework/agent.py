import textwrap
import unittest
from tempfile import NamedTemporaryFile

from lidapy.framework.agent import AgentConfig
from lidapy.framework.shared import FrameworkDependencyService
from lidapy.util.comm import LocalCommunicationProxy
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
        # Remove any singleton instances of FrameworkDependencyService
        # to guarantee that test cases are independent
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()

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
                config = AgentConfig(config_file=tmp.name)

                # Verify that global_param value from config matches
                # expected value
                expected_value = "5"
                actual_value = config.get_global_param("rate_in_hz")

                self.assertEqual(expected_value, actual_value)

                # Verify that global_param value from config matches
                # expected value
                expected_value = "False"
                actual_value = config.get_global_param("use_param_service")

                self.assertEqual(expected_value, actual_value)

                # Verify that none is returned when a global parameter
                # is requested that does not exist
                expected_value = None
                actual_value = config.get_global_param("does_not_exist")

                self.assertEqual(expected_value, actual_value)

                # Verify that parameter_1 value from module_1
                # matches the expected value
                expected_value = "value1"
                actual_value = config.get_param("module_1", "parameter_1")

                self.assertEqual(expected_value, actual_value)

                # Verify that parameter_2 value from module_1
                # matches the expected value
                expected_value = "value2"
                actual_value = config.get_param("module_1", "parameter_2")

                self.assertEqual(expected_value, actual_value)

                # Verify that parameter_1 value from module_2
                # matches the expected value
                expected_value = "value1"
                actual_value = config.get_param("module_2", "parameter_1")

                self.assertEqual(expected_value, actual_value)

                # Verify that parameter_2 value from module_2
                # matches the expected value
                expected_value = "value2"
                actual_value = config.get_param("module_2", "parameter_2")

                self.assertEquals(expected_value, actual_value)

                # Verify that none is returned when a parameter
                # is requested for a non-existent type
                expected_value = None
                actual_value = config.get_param("does_not_exist", "parameter_1")

                self.assertEquals(expected_value, actual_value)

            except Exception as e:
                self.fail("Unexpected Exception: {}".format(e))
