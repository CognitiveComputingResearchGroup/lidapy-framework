import unittest

from lidapy import create_class_instance, LOG_LEVEL_DEBUG
from lidapy import ConsoleLogger


class FunctionsTestCases(unittest.TestCase):
    def test_create_class_instance(self):
        logger = create_class_instance("lidapy.ConsoleLogger")
        self.assertIsInstance(logger, ConsoleLogger)

        # Test positional args
        logger = create_class_instance("lidapy.ConsoleLogger", LOG_LEVEL_DEBUG)
        self.assertIsInstance(logger, ConsoleLogger)
        self.assertEqual(logger.log_level, LOG_LEVEL_DEBUG)

        # Test keyword args
        logger = create_class_instance("lidapy.ConsoleLogger", log_level=LOG_LEVEL_DEBUG)
        self.assertIsInstance(logger, ConsoleLogger)
        self.assertEqual(logger.log_level, LOG_LEVEL_DEBUG)
