import unittest

from lidapy.framework.agent import AgentConfig
from lidapy.framework.shared import FrameworkDependencyService
from lidapy.module.action_selection import ActionSelection
from lidapy.util.comm import LocalCommunicationProxy
from lidapy.util.logger import ConsoleLogger
from lidapy.util.meta import Singleton


class ActionSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Remove any singleton instances of FrameworkDependencyService
        # to guarantee that test cases are independent
        Singleton._remove(FrameworkDependencyService)
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()

        config = AgentConfig(config_file_override=True)
        config.set_global_param("rate_in_hz", 10)

        config.set_param("action_selection", "max_queue_size", 100)
        config.set_param("action_selection", "candidate_behaviors_max_queue_size", 75)
        config.set_param("action_selection", "selected_behaviors_max_queue_size", 75)

        fd["config"] = config

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        module = ActionSelection()

        self.assertEqual(module.default_max_queue_size, 100)

        self.assertEqual(module.candidate_behaviors_max_queue_size, 75)
        self.assertEqual(module.candidate_behaviors_queue.max_queue_size, 75)

        self.assertEqual(module.selected_behaviors_max_queue_size, 75)
        self.assertEqual(module.selected_behaviors_queue.max_queue_size, 75)

        self.assertIsNotNone(module.behavior_net)

    def test_initialize(self):
        module = ActionSelection()

        # Invoke the initialize method and verify side-effects
        module.initialize()

        # Verify subscribers
        expected_subscribers = [module.CANDIDATE_BEHAVIORS_TOPIC_NAME,
                                module.GLOBAL_BROADCAST_TOPIC_NAME]
        for s in expected_subscribers:
            self.assertTrue(s in module.subscribers)

        # Verify publishers
        expected_publishers = [module.SELECTED_BEHAVIORS_TOPIC_NAME]
        for p in expected_publishers:
            self.assertTrue(p in module.publishers)

        # Verify background tasks
        expected_background_tasks = ["behavior_selector",
                                     "behavior_publisher",
                                     "candidate_behaviors_receiver",
                                     "learner"]
        for b in expected_background_tasks:
            self.assertTrue(b in module.background_tasks)

    def test_finalize(self):
        pass

    def test_update_status(self):
        pass

    def test_receive_candidates(self):
        pass

    def test_select_behavior_from_candidates(self):
        pass

    def test_publish_selected_behavior(self):
        pass

    def test_learn(self):
        pass
