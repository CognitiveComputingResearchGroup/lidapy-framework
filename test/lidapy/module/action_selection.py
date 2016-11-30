import unittest

from lidapy.framework.agent import AgentConfig
from lidapy.framework.shared import FrameworkDependencyService
from lidapy.module.action_selection import ActionSelection, BehaviorNetwork
from lidapy.module.procedural_memory import Behavior
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
        expected_background_tasks = ["receiver",
                                     "selector",
                                     "publisher",
                                     "decayer",
                                     "learner"]
        for b in expected_background_tasks:
            self.assertTrue(b in module.background_tasks)

    def test_receive_candidates(self):
        module = ActionSelection()
        module.initialize()

        # Manually add a message to the candidate behaviors message queue
        candidate_behaviors = {Behavior(n) for n in range(10)}

        pub = module.CANDIDATE_BEHAVIORS.create_publisher()
        pub.publish(candidate_behaviors)

        # Manually execute the subscriber callback once
        module.subscribers[module.CANDIDATE_BEHAVIORS_TOPIC_NAME]._listener_func()

        module.receive_candidates()

        self.assertEqual(len(module.candidate_behaviors_queue), 1)
        self.assertEqual(module.candidate_behaviors_queue.pop(), candidate_behaviors)

    def test_basic_select_behavior(self):
        module = ActionSelection()
        module.initialize()

        candidate_behaviors = [Behavior(n) for n in range(10)]
        candidate_behaviors[0].activation = 1.0
        for behavior in candidate_behaviors[1:]:
            behavior.activation = 0.5

        # Manually add candidate behaviors to candidate behaviors message queue
        module.candidate_behaviors_queue.push(candidate_behaviors)

        module.select_behavior_from_candidates()

        # Verify that candidate behaviors removed from candidate_behaviors_queue
        self.assertEqual(len(module.candidate_behaviors_queue), 0)

        # Verify that a candidate behavior was selected and placed onto the
        # selected_behaviors_queue
        self.assertEqual(len(module.selected_behaviors_queue), 1)

        # Verify that the behavior with the highest activation was selected
        self.assertEqual(module.selected_behaviors_queue.pop(), candidate_behaviors[0])

    def test_select_behavior_no_candidates(self):
        module = ActionSelection()
        module.initialize()

        try:
            module.select_behavior_from_candidates()
        except Exception:
            self.fail("Unexpected exception")

        self.assertEqual(len(module.candidate_behaviors_queue), 0)
        self.assertEqual(len(module.selected_behaviors_queue), 0)

    def test_select_behavior_no_match(self):
        module = ActionSelection()
        module.initialize()

        candidate_behaviors = [Behavior(n) for n in range(10)]
        candidate_behaviors[0].activation = 0.75
        for behavior in candidate_behaviors[1:]:
            behavior.activation = 0.0

        # Manually add candidate behaviors to candidate behaviors message queue
        module.candidate_behaviors_queue.push(candidate_behaviors)

        module.select_behavior_from_candidates()

        # Verify that candidate behaviors removed from candidate_behaviors_queue
        self.assertEqual(len(module.candidate_behaviors_queue), 0)

        # Verify that behavior NOT selected
        self.assertEqual(len(module.selected_behaviors_queue), 0)

        # Verify that the candidate threshold was reduced
        self.assertLess(module.behavior_net.candidate_threshold, module.behavior_net.initial_candidate_threshold)

        last_threshold = module.behavior_net.candidate_threshold
        while last_threshold > 0 and len(module.selected_behaviors_queue):
            module.select_behavior_from_candidates()

            if len(module.selected_behaviors_queue) == 0:
                self.assertLess(module.selected_behaviors_queue, last_threshold)
            else:
                self.assertEqual(module.behavior_net.candidate_threshold,
                                 module.behavior_net.initial_candidate_threshold)

            last_threshold = module.behavior_net.candidate_threshold

            if len(module.selected_behaviors_queue) > 0:
                candidate_behaviors = [Behavior(n) for n in range(10)]

                # Manually add candidate behaviors to candidate behaviors message queue
                module.candidate_behaviors_queue.push(candidate_behaviors)

    def test_publish_selected_behavior(self):
        module = ActionSelection()
        module.initialize()

        selected_behavior = Behavior(1)

        # Manually add a behavior to the selected_behaviors_queue
        module.selected_behaviors_queue.push(selected_behavior)

        module.publish_selected_behavior()

        # Verify that behavior removed from the selected_behaviors_queue
        self.assertEqual(len(module.selected_behaviors_queue), 0)

        # Create a subscriber to the "selected_behaviors" topic
        sub = module.SELECTED_BEHAVIORS.create_subscriber()

        # Manually run the listener callback once to retrieve the message
        # into the subscribers temporary queue
        sub._listener_func()

        # Verify that the retrieved message matches the selected_behavior
        # that was previously published
        self.assertEqual(sub.get_next_msg(), selected_behavior)

    def test_learn(self):
        pass

        # def test_start(self):
        #     module = ActionSelection()
        #
        #     starter_dict = dict()
        #
        #     module.receiver_task = FrameworkBackgroundTask(name="receiver", callback=start_check)
        #     module.selector_task = FrameworkBackgroundTask(name="selector", callback=start_check)
        #     module.publisher_task = FrameworkBackgroundTask(name="publisher", callback=start_check)
        #     module.decayer_task = FrameworkBackgroundTask(name="decayer", callback=start_check)
        #     module.learner_task = FrameworkBackgroundTask(name="learner", callback=start_check)
        #
        #     module.start()
        #
        #     while len(starter_dict) < 5:
        #         sleep(.1)
        #
        #     self.assertEquals(len(starter_dict), 5)


class BehaviorNetworkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Remove any singleton instances of FrameworkDependencyService
        # to guarantee that test cases are independent
        Singleton._remove(FrameworkDependencyService)
        fd = FrameworkDependencyService(allow_overrides=True)

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()

        config = AgentConfig(config_file_override=True)
        config.set_global_param("rate_in_hz", 10)

        config.set_param("action_selection", "initial_candidate_threshold", 0.9)
        config.set_param("action_selection", "broadcast_excitation_factor", 0.05)
        config.set_param("action_selection", "successor_excitation_factor", 0.04)
        config.set_param("action_selection", "predecessor_excitation_factor", 0.1)
        config.set_param("action_selection", "conflictor_excitation_factor", 0.05)
        config.set_param("action_selection", "context_satisfaction_threshold", 0.0)

        fd["config"] = config

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        config = FrameworkDependencyService()["config"]
        config.set_param("action_selection", "initial_candidate_threshold", 0.1)
        config.set_param("action_selection", "broadcast_excitation_factor", 0.2)
        config.set_param("action_selection", "successor_excitation_factor", 0.3)
        config.set_param("action_selection", "predecessor_excitation_factor", 0.4)
        config.set_param("action_selection", "conflictor_excitation_factor", 0.5)
        config.set_param("action_selection", "context_satisfaction_threshold", 0.6)

        bn = BehaviorNetwork()

        # Verify parameters pulled from configuration
        self.assertEqual(bn.initial_candidate_threshold, 0.1)
        self.assertEqual(bn.broadcast_excitation_factor, 0.2)
        self.assertEqual(bn.successor_excitation_factor, 0.3)
        self.assertEqual(bn.predecessor_excitation_factor, 0.4)
        self.assertEqual(bn.conflictor_excitation_factor, 0.5)
        self.assertEqual(bn.context_satisfaction_threshold, 0.6)
        self.assertEqual(bn.candidate_threshold, bn.initial_candidate_threshold)

        self.assertNotEqual(bn.candidate_threshold_decay_strategy, None)
        self.assertNotEqual(bn.behavior_decay_strategy, None)

        self.assertNotEqual(bn.behaviors, None)
