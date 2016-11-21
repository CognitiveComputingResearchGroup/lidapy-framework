import unittest
import threading

from lidapy.framework.agent import AgentConfig
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.shared import FrameworkDependencyService, CognitiveContentStructure, CognitiveContent


class AttentionCodeletTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_codelet(self):

        def some_function(ccs):
            found_content = []
            for cc in ccs:
                if int(cc.value) % 2 == 0:
                    found_content.append(cc)

            return found_content

        csm = CognitiveContentStructure()
        csm += CognitiveContent("1")
        csm += CognitiveContent("2")
        csm += CognitiveContent("3")
        csm += CognitiveContent("4")

        ccq = CognitiveContentStructure()
        ccq += CognitiveContent("5")
        ccq += CognitiveContent("6")
        ccq += CognitiveContent("7")
        ccq += CognitiveContent("8")

        # At a high-level an AttentionCodelet is expected to do the following:
        #
        # (1) Look for sought content in a workspace buffer
        # (2) If sought content is found, create a "coalition" from the sought content
        # (3) Publish the new coalition to the workspace_coalitions topic
        codelet = AttentionCodelet(name="attn_codelet",
                                   search_function=some_function,
                                   search_targets=[csm, ccq],
                                   base_level_activation=0.5,
                                   removal_threshold=0.0,
                                   exec_count=1)

        # Testing for expected codelet name
        self.assertEqual(codelet.name, "attn_codelet")

        # Testing for expected search function
        self.assertEqual(codelet.search_function, some_function)

        # Test for expected targets of search.
        expected_targets = [csm, ccq]
        self.assertEqual(len(expected_targets), len(codelet.search_targets))
        self.assertEqual(set(expected_targets), set(codelet.search_targets))

        # Testing for expected initial base_level_activation
        self.assertAlmostEqual(codelet.base_level_activation, 0.5)

        # Testing for expected removal_threshold
        self.assertAlmostEqual(codelet.removal_threshold, 0.0)

        # Starting codelet as a background thread
        codelet.start()

        if threading.currentThread() == codelet.parent_thread:
            codelet.wait_until_complete()

            # The expected activation is initial activation - exec_count * (slope / rate_in_hz)
            for cc in ccs:
                self.assertAlmostEqual(cc.activation, 0.9)

        # The FrameworkAttentionCodelet must be an Activatable
        #

        # Need to verify that the coalition created by the attention codelet
        # contains the expected values
        # Assuming: Coalition is a cognitive content structure
        # Need to stub out message publication to make sure that the message
        # published by the codelet is received (without needing ros)

        GLOBAL_WORKSPACE_TOPIC = FrameworkTopic("global_workspace")

        coalition = CognitiveContentStructure()
        coalition += CognitiveContent("2")
        coalition += CognitiveContent("4")
        coalition += CognitiveContent("6")
        coalition += CognitiveContent("8")

        GLOBAL_WORKSPACE_TOPIC.publisher.publish(coalition)

        # Need StubFrameworkTopicPublisher
        # Need to add a add_publisher implementation in StubCommunicationProxy that
        # returns a StubFrameworkTopicPublisher


        # Need to add status tests "RUNNING", "ERROR", etc.
