import unittest
import threading

from lidapy.framework.agent import AgentConfig
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.process import FrameworkThread, FrameworkBackgroundTask, BackgroundDecayTask
from lidapy.framework.shared import FrameworkDependencyService, Activatable, CognitiveContentStructure, CognitiveContent
from lidapy.framework.strategy import LinearDecayStrategy
from lidapy.util.comm import LocalCommunicationProxy
from lidapy.util.logger import ConsoleLogger
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

    def test_thread(self):

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

    def test_background_task(self):
        class CallBack():
            def __init__(self):
                self.count = 0

            def __call__(self, *args, **kwargs):
                self.count += 1

        callback_cls = CallBack()

        bg_task = FrameworkBackgroundTask(name="BackgroundTask", callback=callback_cls, exec_count=100)
        bg_task.start()
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

    def test_init(self):
        name = "linearDecayTask"
        strategy = LinearDecayStrategy(slope=0.1)
        target = Activatable()
        exec_count = 17

        def a_getter(target):
            return target.base_level_activation

        def a_setter(target, value):
            target.base_level_activation = value

        getter = a_getter
        setter = a_setter

        task = BackgroundDecayTask(name=name, strategy=strategy, target=target, getter=getter, setter=setter,
                                   exec_count=exec_count)

        self.assertEqual(task.name, name)
        self.assertEqual(task.strategy, strategy)
        self.assertEqual(task.target, target)
        self.assertEqual(task.getter, getter)
        self.assertEqual(task.setter, setter)
        self.assertEqual(task.exec_count, exec_count)

        # Test for required values
        with self.assertRaises(ValueError):
            BackgroundDecayTask(name=None, strategy=strategy, target=target, getter=getter, setter=setter,
                                exec_count=exec_count)

        with self.assertRaises(ValueError):
            BackgroundDecayTask(name=name, strategy=None, target=target, getter=getter, setter=setter,
                                exec_count=exec_count)

        with self.assertRaises(ValueError):
            BackgroundDecayTask(name=name, strategy=strategy, target=None, getter=getter, setter=setter,
                                exec_count=exec_count)

        # Test for defaults
        task = BackgroundDecayTask(name=name, strategy=strategy, target=target)

        self.assertEqual(task.getter, BackgroundDecayTask.default_getter)
        self.assertEqual(task.setter, BackgroundDecayTask.default_setter)
        self.assertEqual(task.exec_count, -1)

    def test_default_decay_task(self):
        ccs = CognitiveContentStructure()

        cc_list = [CognitiveContent(value) for value in range(10)]

        for cc in cc_list:
            cc.activation = 1.0
            ccs.insert(cc)

        linear_decay = LinearDecayStrategy(slope=0.1)
        decay_task = BackgroundDecayTask(name="linearDecay", strategy=linear_decay, target=ccs, exec_count=10)
        decay_task.start()
        decay_task.wait_until_complete()

        # The expected activation is initial activation - exec_count * (slope / rate_in_hz)
        for cc in ccs:
            self.assertAlmostEqual(cc.activation, 0.9)

    def test_custom_decay_task(self):
        ccs = CognitiveContentStructure()

        cc_list = [CognitiveContent(value) for value in range(10)]

        initial_activation = 1.0
        initial_base_level_activation = 1.0

        for cc in cc_list:
            cc.activation = initial_activation
            cc.base_level_activation = initial_base_level_activation
            ccs.insert(cc)

        linear_decay = LinearDecayStrategy(slope=0.1)

        def bla_getter(target):
            return target.base_level_activation

        def bla_setter(target, value):
            target.base_level_activation(value)

        decay_task = BackgroundDecayTask(name="linearDecay", strategy=linear_decay,
                                         target=ccs, getter=bla_getter, setter=bla_setter,
                                         exec_count=10)

        decay_task.start()
        decay_task.wait_until_complete()

        for cc in ccs:
            # Activation should be unchanged
            self.assertAlmostEqual(cc.activation, initial_activation)

            # The expected base level activation is
            #     initial activation - exec_count * (slope / rate_in_hz)
            self.assertAlmostEqual(cc.base_level_activation, initial_base_level_activation)
