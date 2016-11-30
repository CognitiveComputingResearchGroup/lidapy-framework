import unittest

from lidapy.util.functions import create_class_instance
from lidapy.util.logger import ConsoleLogger, DEBUG


class FunctionsTestCases(unittest.TestCase):
    def test_create_class_instance(self):
        logger = create_class_instance("lidapy.util.logger.ConsoleLogger")
        self.assertIsInstance(logger, ConsoleLogger)

        # Test positional args
        logger = create_class_instance("lidapy.util.logger.ConsoleLogger", DEBUG)
        self.assertIsInstance(logger, ConsoleLogger)
        self.assertEqual(logger.log_level, DEBUG)

        # Test keyword args
        logger = create_class_instance("lidapy.util.logger.ConsoleLogger", log_level=DEBUG)
        self.assertIsInstance(logger, ConsoleLogger)
        self.assertEqual(logger.log_level, DEBUG)
