import unittest

import lidapy
from lidapy import Behavior
from lidapy import Config
from lidapy.modules import ActionSelection, SELECTED_BEHAVIORS_TOPIC
from lidapy.modules import CANDIDATE_BEHAVIORS_TOPIC


class ActionSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):

        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')
        config.set_param('rate_in_hz', 10)

        lidapy.init(config=config, process_name='test')

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        pass

    def test_initialize(self):
        pass

    # def test_receive_candidates(self):
    #     module = ActionSelection()
    #     module.initialize()
    #
    #     # Manually add a message to the candidate behaviors message queue
    #     candidate_behaviors = {Behavior(n) for n in range(10)}
    #
    #     CANDIDATE_BEHAVIORS_TOPIC.receive(timeout=1)
    #     CANDIDATE_BEHAVIORS_TOPIC.send(candidate_behaviors)
    #     module.receive_candidates()
    #
    #     self.assertEqual(len(module.candidate_behaviors), 1)

    def test_basic_select_behavior(self):
        module = ActionSelection()
        module.initialize()

        candidate_behaviors = [Behavior(n) for n in range(10)]
        candidate_behaviors[0].activation = 1.0
        for behavior in candidate_behaviors[1:]:
            behavior.activation = 0.5

        # Manually add candidate behaviors to candidate behaviors message queue
        module.candidate_behaviors.push(candidate_behaviors)
        module.select_behavior()

        # Verify that candidate behaviors removed from candidate_behaviors
        self.assertEqual(len(module.candidate_behaviors), 0)

        # Verify that a candidate behavior was selected and placed onto the
        # selected_behaviors
        self.assertEqual(len(module.selected_behaviors), 1)

        # Verify that the behavior with the highest activation was selected
        self.assertEqual(module.selected_behaviors.pop(), candidate_behaviors[0])

    def test_select_behavior_no_candidates(self):
        module = ActionSelection()
        module.initialize()

        try:
            module.select_behavior()
        except Exception:
            self.fail('Unexpected exception')

        self.assertEqual(len(module.candidate_behaviors), 0)
        self.assertEqual(len(module.selected_behaviors), 0)

    def test_select_behavior_no_match(self):
        module = ActionSelection()
        module.initialize()

        candidate_behaviors = [Behavior(n) for n in range(10)]
        candidate_behaviors[0].activation = 0.75
        for behavior in candidate_behaviors[1:]:
            behavior.activation = 0.0

        # Manually add candidate behaviors to candidate behaviors message queue
        module.candidate_behaviors.push(candidate_behaviors)
        module.select_behavior()

        # Verify that candidate behaviors removed from candidate_behaviors
        self.assertEqual(len(module.candidate_behaviors), 0)

        # Verify that behavior NOT selected
        self.assertEqual(len(module.selected_behaviors), 0)

        # Verify that the candidate threshold was reduced
        self.assertLess(module.behavior_net.candidate_threshold, module.behavior_net.initial_candidate_threshold)

        last_threshold = module.behavior_net.candidate_threshold
        while last_threshold > 0 and len(module.selected_behaviors):
            module.select_behavior()

            if len(module.selected_behaviors) == 0:
                self.assertLess(module.selected_behaviors, last_threshold)
            else:
                self.assertEqual(module.behavior_net.candidate_threshold,
                                 module.behavior_net.initial_candidate_threshold)

            last_threshold = module.behavior_net.candidate_threshold

            if len(module.selected_behaviors) > 0:
                candidate_behaviors = [Behavior(n) for n in range(10)]

                # Manually add candidate behaviors to candidate behaviors message queue
                module.candidate_behaviors.push(candidate_behaviors)

    # def test_publish_selected_behavior(self):
    #     module = ActionSelection()
    #     module.initialize()
    #
    #     SELECTED_BEHAVIORS_TOPIC.receive(0.001)
    #
    #     selected_behavior = Behavior(1)
    #
    #     # Manually add a behavior to the selected_behaviors
    #     module.selected_behaviors.push(selected_behavior)
    #     module.publish_selected_behavior()
    #
    #     # Verify that behavior removed from the selected_behaviors
    #     self.assertEqual(len(module.selected_behaviors), 0)
    #
    #     # Verify that the retrieved message matches the selected_behavior
    #     # that was previously published
    #     received_behavior = SELECTED_BEHAVIORS_TOPIC.receive()
    #     self.assertEqual(received_behavior.unique_id, selected_behavior.unique_id)

    def test_learn(self):
        pass

