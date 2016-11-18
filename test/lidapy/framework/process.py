import unittest

from lidapy.codelet.attention_codelet import AttentionCodelet
from lidapy.framework.agent import AgentConfig
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.process import FrameworkThread, FrameworkBackgroundTask, BackgroundDecayTask
from lidapy.framework.shared import FrameworkDependencyService, CognitiveContentStructure, CognitiveContent
from lidapy.framework.strategy import LinearDecayStrategy
from lidapy.util.comm import LocalCommunicationProxy
from lidapy.util.logger import ConsoleLogger
import threading
from lidapy.util.meta import Singleton


class FrameworkThreadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Remove any singleton instances of FrameworkDependencyService
        # to guarantee that test cases are independent
        Singleton._remove(FrameworkDependencyService)
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()

        config = AgentConfig(config_file_override=True)
        config.set_global_param("rate_in_hz", 1000)

        fd["config"] = config

    @classmethod
    def tearDownClass(cls):
        pass

    def test(self):

        class CallBack():
            def __init__(self):
                self.count = 0

            def __call__(self, *args, **kwargs):
                self.count += 1

        callback_cls = CallBack()

        try:

            task = FrameworkThread(name="test_task",
                                   callback=callback_cls,
                                   exec_count=100)

            # Start task in thread
            task.start()

            # Wait for thread to complete
            task.join()

            # Verify that execution count matches expected number of executions
            self.assertEqual(100, callback_cls.count)

        except Exception as e:
            self.fail("Unexpected Exception: {}".format(e))


class BackgroundTaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Remove any singleton instances of FrameworkDependencyService
        # to guarantee that test cases are independent
        Singleton._remove(FrameworkDependencyService)
        fd = FrameworkDependencyService()

        fd["logger"] = ConsoleLogger()
        fd["ipc_proxy"] = LocalCommunicationProxy()

        config = AgentConfig(config_file_override=True)
        config.set_global_param("rate_in_hz", 1000)

        fd["config"] = config

    @classmethod
    def tearDownClass(cls):
        pass

    def test_decay_task(self):
        class CallBack():
            def __init__(self):
                self.count = 0

            def __call__(self, *args, **kwargs):
                self.count += 1

        callback_cls = CallBack()

        bg_task = FrameworkBackgroundTask(name="BackgroundTask", callback=callback_cls, exec_count=100)

        bg_task.start()

        if threading.currentThread() == bg_task.parent_thread:
            bg_task.wait_until_complete()

            # Verify that execution count matches expected number of executions
            self.assertEqual(100, callback_cls.count)


class BackgroundDecayTaskTest(unittest.TestCase):
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

        fd["config"] = config

    @classmethod
    def tearDownClass(cls):
        pass

    def test_decay_task(self):
        ccs = CognitiveContentStructure()

        cc_list = [CognitiveContent(1),
                   CognitiveContent(2),
                   CognitiveContent(3),
                   CognitiveContent(4),
                   CognitiveContent(5),
                   CognitiveContent(6),
                   CognitiveContent(7),
                   CognitiveContent(8),
                   CognitiveContent(9),
                   CognitiveContent(10),
                   ]

        for cc in cc_list:
            cc.activation = 1.0
            ccs.insert(cc)

        linear_decay = LinearDecayStrategy(slope=0.1)
        decay_task = BackgroundDecayTask(name="linearDecay", strategy=linear_decay, target=ccs, exec_count=10)

        decay_task.start()

        if threading.currentThread() == decay_task.parent_thread:
            decay_task.wait_until_complete()

            # The expected activation is initial activation - exec_count * (slope / rate_in_hz)
            for cc in ccs:
                self.assertAlmostEqual(cc.activation, 0.9)


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
