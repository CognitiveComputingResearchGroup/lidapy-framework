import tempfile
import textwrap
import time
import unittest

import lidapy
from lidapy import Activatable
from lidapy import CognitiveContent
from lidapy import Config
from lidapy import DecayTask
from lidapy import MsgListener
from lidapy import Task
from lidapy import Topic
from lidapy import create_class_instance
from lidapy.strategy import LinearDecayStrategy, MAX_ACTIVATION, SigmoidExciteStrategy, MIN_ACTIVATION, \
    SigmoidDecayStrategy, LinearExciteStrategy


class ConfigTest(unittest.TestCase):
    def test_config_with_file_path(self):
        # Test that non-existent configuration file returns IOError
        with self.assertRaises(IOError):
            Config(file_path='does_not_exist')

        # Use the TemporaryFile context manager for easy clean-up
        with tempfile.NamedTemporaryFile() as tmp:
            # Load temporary configuration file from a string
            config_file = \
                '''
                [global]
                p1 = gv1

                [section_1]
                p1 = s1v1
                p2 = s1v2

                [section_2]
                p1 = s2v1
                p2 = s2v2
                '''
            cleansed_test_config = textwrap.dedent(config_file).lstrip()
            tmp.write(cleansed_test_config)
            tmp.flush()

            config = Config(file_path=tmp.name)

            self._check_values(config)

    def test_config_no_file_path(self):
        config = Config()

        config.set_param(name='p1', value='gv1')
        config.set_param(name='p1', value='s1v1', section='section_1')
        config.set_param(name='p2', value='s1v2', section='section_1')
        config.set_param(name='p1', value='s2v1', section='section_2')
        config.set_param(name='p2', value='s2v2', section='section_2')

        self._check_values(config)

    def _check_values(self, config):
        # Verify that global value from config matches expected value
        expected = 'gv1'
        actual = config.get_param('p1')

        self.assertEqual(expected, actual)

        # Verify that p1 value from section_1 matches the expected value
        expected = 's1v1'
        actual = config.get_param('p1', section='section_1')

        self.assertEqual(expected, actual)

        # Verify that p2 value from section_1 matches the expected value
        expected = 's1v2'
        actual = config.get_param('p2', section='section_1')

        self.assertEqual(expected, actual)

        # Verify that p1 value from section_2 matches the expected value
        expected = 's2v1'
        actual = config.get_param('p1', section='section_2')

        self.assertEqual(expected, actual)

        # Verify that p2 value from section_2 matches the expected value
        expected = 's2v2'
        actual = config.get_param('p2', section='section_2')

        self.assertEqual(expected, actual)

        # Verify that None is returned when a parameter
        # is requested for a non-existent type with no default
        expected = None
        actual = config.get_param('p1', section='does_not_exist')

        self.assertEquals(expected, actual)

        # Verify that None is returned when a global parameter
        # is requested that does not exist
        expected = None
        actual = config.get_param('does_not_exist')

        self.assertEqual(expected, actual)

        # Verify that None is returned when a section parameter
        # is requested that does not exist
        expected = None
        actual = config.get_param('does_not_exist', section='section_1')

        self.assertEqual(expected, actual)

        # Verify default returned when non-existent parameter requested
        expected = 'd1'
        actual = config.get_param('does_not_exist', section='section_1', default=expected)

        self.assertEquals(expected, actual)

        # Verify default returned when non-existent parameter requested
        expected = 'd1'
        actual = config.get_param('p1', section='does_not_exist', default=expected)

        self.assertEquals(expected, actual)

        # def test_config_with_param_service(self):
        #     # Use the TemporaryFile context manager for easy clean-up
        #     with tempfile.NamedTemporaryFile() as tmp:
        #         # Load temporary configuration file from a string
        #         config_file = \
        #             '''
        #             [global]
        #             p1 = gv1
        #
        #             [section_1]
        #             p1 = s1v1
        #             p2 = s1v2
        #
        #             [section_2]
        #             p1 = s2v1
        #             p2 = s2v2
        #             '''
        #         cleansed_test_config = textwrap.dedent(config_file).lstrip()
        #         tmp.write(cleansed_test_config)
        #         tmp.flush()
        #
        #         config = Config(file_path=tmp.name, use_param_service=True)
        #         self._check_values(config)


class ActivatableTest(unittest.TestCase):
    def test_init(self):
        # Verify default initial values
        a = Activatable()
        self.assertEqual(a.activation, a.DEFAULT_ACTIVATION)
        self.assertEqual(a.base_level_activation, a.DEFAULT_BASE_LEVEL_ACTIVATION)
        self.assertEqual(a.incentive_salience, a.DEFAULT_INCENTIVE_SALIENCE)
        self.assertEqual(a.removal_threshold, a.DEFAULT_REMOVAL_THRESHOLD)

        # Verify that initial activation can be set during initialization
        a = Activatable(activation=0.25, base_level_activation=0.05, incentive_salience=0.5, removal_threshold=0.01)
        self.assertEqual(a.activation, 0.25)
        self.assertEqual(a.base_level_activation, 0.05)
        self.assertEqual(a.incentive_salience, 0.5)
        self.assertEqual(a.removal_threshold, 0.01)

    def test_set_props(self):
        a = Activatable()
        a.activation = 0.25
        a.base_level_activation = 0.05
        a.incentive_salience = 0.5
        a.removal_threshold = 0.01

        # Verify small change of activation and incentive salience within
        # expected range bounds works
        a = Activatable()
        a.activation += 0.25
        self.assertEqual(a.activation, 0.25)

        a.activation -= 0.1
        self.assertEqual(a.activation, 0.15)

        a = Activatable()
        a.incentive_salience += 0.25
        self.assertEqual(a.incentive_salience, 0.25)

        a.incentive_salience -= 0.1
        self.assertEqual(a.incentive_salience, 0.15)

        a = Activatable()
        a.base_level_activation += 0.25
        self.assertEqual(a.base_level_activation, 0.25)

        a.base_level_activation -= 0.1
        self.assertEqual(a.base_level_activation, 0.15)

        # Verify that reduction of activation and incentive salience and base_level_activation
        # to below lower bound is scaled to lower bound
        a = Activatable()
        a.activation -= 0.25
        self.assertEqual(a.activation, 0.0)

        a = Activatable()
        a.incentive_salience -= 0.25
        self.assertEqual(a.incentive_salience, 0.0)

        a = Activatable()
        a.base_level_activation -= 0.25
        self.assertEqual(a.base_level_activation, 0.0)

        # Verify that increase of activation and incentive salience and base_level_activation
        # to above upper bound is scaled to upper bound
        a = Activatable()
        a.activation += 2.0
        self.assertEqual(a.activation, 1.0)

        a = Activatable()
        a.incentive_salience += 2.0
        self.assertEqual(a.incentive_salience, 1.0)

        a = Activatable()
        a.base_level_activation += 2.0
        self.assertEqual(a.base_level_activation, 1.0)


class LocalCommunicationProxyTest(unittest.TestCase):
    # Access to internal LidaPy globals for testing low-level operations
    _var = None

    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')
        config.set_param('rate_in_hz', '10')

        cls._var = lidapy.init(config=config, process_name='test')

    def test_publisher(self):
        def callback():
            pass

        pub = self._var.ipc.get_publisher(topic_name='topic', preprocessor=callback)

        self.assertIsInstance(pub.__class__, lidapy.LocalTopicPublisher.__class__)
        self.assertEqual(pub.topic_name, 'topic')
        self.assertEqual(pub.preprocessor, callback)

    def test_subscriber(self):
        def callback():
            pass

        sub = self._var.ipc.get_subscriber(topic_name='topic',
                                           postprocessor=callback)

        self.assertIsInstance(sub.__class__, lidapy.LocalTopicSubscriber.__class__)
        self.assertEqual(sub.topic_name, 'topic')
        self.assertEqual(sub.postprocessor, callback)

    def test_service(self):
        # Not implemented
        pass

    def test_service_proxy(self):
        # Not implemented
        pass


class LocalMessageQueueTest(unittest.TestCase):
    def test_init(self):
        q = lidapy.LocalMessageQueue(name='queue_name', msg_type=Activatable, max_queue_size=15)

        self.assertEqual(q.name, 'queue_name')
        self.assertEqual(q.msg_type, Activatable)
        self.assertEqual(q.max_queue_size, 15)

    def test(self):

        q = lidapy.LocalMessageQueue()

        msgs = [str(x) for x in range(10)]
        for msg in msgs:
            q.push(msg)

        # Verify that length of queue equals messages pushed onto it
        self.assertEqual(len(msgs), len(q))

        # Verify that peek returns the first message
        self.assertEqual(q.peek(), msgs[0])

        # Verify that pop returns the next message in FIFO order removes it from the queue
        while len(q) > 0:
            expected = msgs.pop(0)
            actual = q.pop()

            if expected != actual:
                self.fail('Expected {} != Actual {}'.format(expected, actual))

        # Verify queue is empty after popping all values
        self.assertEqual(len(q), 0)
        self.assertEqual(len(msgs), 0)

    def test_event(self):
        q = lidapy.LocalMessageQueue()

        self.assertEqual(q.event.is_set(), False)
        q.push('msg')
        self.assertEqual(q.event.is_set(), True)
        q.peek()
        self.assertEqual(q.event.is_set(), True)
        # First pop does not clear event.  Only cleared
        # when pop from empty queue
        q.pop()
        q.pop()
        self.assertEqual(q.event.is_set(), False)


class LocalTopicTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')

        lidapy.init(config=config, process_name='test')

    def test_init(self):
        def custom_pre():
            return 'custom_pre'

        def custom_post():
            return 'custom_post'

        # Test initializer sets attributes correctly
        topic = Topic(name='topic', msg_type=Activatable, queue_size=50, preprocessor=custom_pre,
                      postprocessor=custom_post)

        self.assertEqual(topic.name, 'topic')
        self.assertEqual(topic.msg_type, Activatable)
        self.assertEqual(topic.queue_size, 50)
        self.assertEqual(topic.preprocessor, custom_pre)
        self.assertEqual(topic.postprocessor, custom_post)

    def test_create_pub_sub(self):
        topic = Topic(name='topic')

        topic._init_publisher()
        topic._init_subscriber()

        self.assertEqual(type(topic._publisher), lidapy.LocalTopicPublisher)
        self.assertEqual(type(topic._subscriber), lidapy.LocalTopicSubscriber)

        self.assertEqual(topic._publisher.msg_type, topic._subscriber.msg_type)

    def test_preprocessor(self):
        topic = Topic('topic', preprocessor=lambda x: str(x) + '_processed')

        listener = topic.add_listener()

        recv_msg = None
        while recv_msg is None:
            topic.send('1')
            recv_msg = listener.receive(timeout=.001)

        if '_processed' not in recv_msg:
            self.fail('Failed to find expected string')

    def test_postprocessor(self):
        topic = Topic('topic', postprocessor=lambda x: x.replace('_processed', ''))

        listener = topic.add_listener()

        msg = None
        while msg is None:
            topic.send('1_processed')
            msg = listener.receive(timeout=.001)

        self.assertEqual('1', msg)


class MsgListenerTest(unittest.TestCase):
    def test(self):
        listener = MsgListener(queue_size=10)

        # Test initial state
        self.assertEqual(listener.event.is_set(), False)
        self.assertEqual(len(listener.msg_queue), 0)
        self.assertEqual(listener.receive(timeout=0.001), None)

        listener.notify('msg')

        # Test notify sets event and adds message to the message
        # queue for retrieval
        self.assertEqual(listener.event.is_set(), True)
        self.assertEqual(len(listener.msg_queue), 1)

        next_msg = listener.receive(timeout=0.001)

        # Test that next_msg pulls last message from message queue.
        # Also tests that the event is not unset.  This is to handle
        # multiple sets (len(Q) > 1) or potential issues due to
        # multi-threads (read/write race conditions).
        self.assertEqual(next_msg, 'msg')
        self.assertEqual(listener.event.is_set(), True)
        self.assertEqual(len(listener.msg_queue), 0)

        next_msg = listener.receive(timeout=0.001)

        # Verify that subsequent next_msg calls will return
        # None and clear the event.
        self.assertEqual(next_msg, None)
        self.assertEqual(listener.event.is_set(), False)
        self.assertEqual(len(listener.msg_queue), 0)

        # Test multiple messages
        listener.notify('msg1')
        listener.notify('msg2')

        next_msg = listener.receive(timeout=0.001)

        # Test that first message returned (in proper FIFO order)
        # and that the event set is retained because of remaining
        # message.  Also verify that message queue still has a
        # single message.
        self.assertEqual(next_msg, 'msg1')
        self.assertEqual(listener.event.is_set(), True)
        self.assertEqual(len(listener.msg_queue), 1)

        next_msg = listener.receive(timeout=0.001)

        # Test that second message returned and that the event is still
        # set. Also verify that message queue is now empty.
        self.assertEqual(next_msg, 'msg2')
        self.assertEqual(listener.event.is_set(), True)
        self.assertEqual(len(listener.msg_queue), 0)

        next_msg = listener.receive(timeout=0.001)

        # Verify that subsequent next_msg calls will return
        # None and clear the event.
        self.assertEqual(next_msg, None)
        self.assertEqual(listener.event.is_set(), False)
        self.assertEqual(len(listener.msg_queue), 0)


class TopicSubscriberTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')

        lidapy.init(config=config, process_name='test')

    def test_listeners(self):
        n_listeners = 100

        listeners = [MsgListener() for n in xrange(n_listeners)]

        topic = Topic('test_topic')
        topic._init_subscriber()
        for listener in listeners:
            topic._subscriber.add_listener(listener)

        topic.send('msg')

        time.sleep(1)

        for listener in listeners:
            self.assertEqual(1, len(listener.msg_queue))


class LIDAThreadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')

        lidapy.init(config=config, process_name='test')

    def test_thread(self):

        class CallBack(object):
            def __init__(self):
                self.count = 0

            def __call__(self):
                self.count += 1

        callback_cls = CallBack()

        try:

            task = lidapy.LIDAThread(name='test',
                                     callback=callback_cls,
                                     exec_count=100)

            # Start task in thread
            task.start()

            # Wait for thread to complete
            task.join()

            # Verify that execution count matches expected number of executions
            self.assertEqual(100, callback_cls.count)

        except Exception as e:
            self.fail('Unexpected Exception: {}'.format(e))


class TaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')

        lidapy.init(config=config, process_name='test')

    def test_background_task(self):
        class CallBack(object):
            def __init__(self):
                self.count = 0

            def __call__(self):
                self.count += 1

        callback_cls = CallBack()

        bg_task = Task(name='test', callback=callback_cls, exec_count=100)
        bg_task.start()
        bg_task.wait_until_complete()

        # Verify that execution count matches expected number of executions
        self.assertEqual(100, callback_cls.count)


class DecayTaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')
        config.set_param('rate_in_hz', '10')

        lidapy.init(config=config, process_name='test')

    def test_init(self):
        name = 'linearDecayTask'
        strategy = LinearDecayStrategy(slope=0.1)
        target = Activatable()
        exec_count = 17

        def a_getter(act):
            return act.base_level_activation

        def a_setter(act, value):
            act.base_level_activation = value

        getter = a_getter
        setter = a_setter

        task = DecayTask(name=name, strategy=strategy, target=target, getter=getter, setter=setter,
                         exec_count=exec_count)

        self.assertEqual(task.name, name)
        self.assertEqual(task.strategy, strategy)
        self.assertEqual(task.target, target)
        self.assertEqual(task.getter, getter)
        self.assertEqual(task.setter, setter)
        self.assertEqual(task.exec_count, exec_count)

        # Test for defaults
        task = DecayTask(name=name, strategy=strategy, target=target)

        self.assertEqual(task.getter, DecayTask.default_getter)
        self.assertEqual(task.setter, DecayTask.default_setter)
        self.assertEqual(task.exec_count, -1)

    def test_default_decay_task(self):
        cc_list = [CognitiveContent(value) for value in range(10)]

        for cc in cc_list:
            cc.activation = 1.0

        linear_decay = LinearDecayStrategy(slope=0.1)
        decay_task = DecayTask(name='linearDecay', strategy=linear_decay, target=cc_list, exec_count=10)
        decay_task.start()
        decay_task.wait_until_complete()

        # The expected activation is initial activation - exec_count * (slope / rate_in_hz)
        for cc in cc_list:
            self.assertAlmostEqual(cc.activation, 0.9)

    def test_custom_decay_task(self):
        cc_list = [CognitiveContent(value) for value in range(10)]

        initial_activation = 1.0
        initial_base_level_activation = 1.0

        for cc in cc_list:
            cc.activation = initial_activation
            cc.base_level_activation = initial_base_level_activation

        linear_decay = LinearDecayStrategy(slope=0.1)

        def bla_getter(act):
            return act.base_level_activation

        def bla_setter(act, val):
            act.base_level_activation = val

        decay_task = DecayTask(name='linearDecay', strategy=linear_decay,
                               target=cc_list, getter=bla_getter, setter=bla_setter,
                               exec_count=10)

        decay_task.start()
        decay_task.wait_until_complete()

        for cc in cc_list:
            # Activation should be unchanged
            self.assertAlmostEqual(cc.activation, initial_activation)

            # The expected base level activation is
            #     initial activation - exec_count * (slope / rate_in_hz)
            self.assertAlmostEqual(cc.base_level_activation, 0.9)


class LinearExciteStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        strategy = LinearExciteStrategy(slope=0.1)
        self.assertEqual(strategy.slope, 0.1)

        with self.assertRaises(ValueError):
            LinearExciteStrategy(slope=0.0)

        with self.assertRaises(ValueError):
            LinearExciteStrategy(slope=-1.0)

    def test_get_next_value(self):
        strategy = LinearExciteStrategy(slope=0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=0.0, rate_in_hz=-0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=0.0, rate_in_hz=0.0)

        current_activation = 0.0
        for rate in [1.0, 10.0, 100.0, 1000.0]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=rate)
            self.assertAlmostEqual(current_activation + (strategy.slope / rate), next_value)

        for current_activation in [0.0, 0.1, 0.5, 0.75, 0.9]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=1.0)
            self.assertAlmostEqual(current_activation + strategy.slope, next_value)

        # Verify that MAX_ACTIVATION is enforced
        next_value = strategy.get_next_value(current_activation=MAX_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(MAX_ACTIVATION, next_value)


class LinearDecayStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        strategy = LinearDecayStrategy(slope=0.1)
        self.assertEqual(strategy.slope, 0.1)

        with self.assertRaises(ValueError):
            LinearDecayStrategy(slope=0.0)

        with self.assertRaises(ValueError):
            LinearDecayStrategy(slope=-1.0)

    def test_get_next_value(self):
        strategy = LinearDecayStrategy(slope=0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=1.0, rate_in_hz=-0.1)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation=1.0, rate_in_hz=0.0)

        current_activation = 1.0
        for rate in [1.0, 10.0, 100.0, 1000.0]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=rate)
            self.assertAlmostEqual(current_activation - (strategy.slope / rate), next_value)

        for current_activation in [0.1, 0.5, 0.75, 0.9, 1.0]:
            next_value = strategy.get_next_value(current_activation=current_activation, rate_in_hz=1.0)
            self.assertAlmostEqual(current_activation - strategy.slope, next_value)

        # Verify that MIN_ACTIVATION is enforced
        next_value = strategy.get_next_value(current_activation=MIN_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(MIN_ACTIVATION, next_value)


class SigmoidDecayStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_init(self):
        strategy = SigmoidDecayStrategy(rate_multiplier=0.1)
        self.assertEqual(strategy.rate_multiplier, 0.1)

        strategy = SigmoidDecayStrategy()
        self.assertEqual(strategy.rate_multiplier, 1.0)

        with self.assertRaises(ValueError):
            SigmoidDecayStrategy(rate_multiplier=0.0)

        with self.assertRaises(ValueError):
            SigmoidDecayStrategy(rate_multiplier=-1.0)

    def test_get_next_value(self):
        # Verify expected value after decrease starting at activation = 0.5
        strategy = SigmoidDecayStrategy()

        current_activation = 0.5

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.268941, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(2.0)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.119203, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidDecayStrategy(0.5)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.377541, places=5)

        # Verify expected value after decrease starting at activation = MIN_ACTIVATION is
        # still approximately MIN_ACTIVATION
        strategy = SigmoidDecayStrategy()

        next_value = strategy.get_next_value(MIN_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, MIN_ACTIVATION, places=5)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=0.0)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=-1.0)


class SigmoidExciteStrategyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_get_next_value(self):
        # Verify expected value after decrease starting a activation = 0.5
        strategy = SigmoidExciteStrategy()

        current_activation = 0.5

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.731059, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(2.0)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.880797, places=5)

        # Verify expected value after decrease starting a activation = 0.5 with
        # rate_multiplier = 2.0
        strategy = SigmoidExciteStrategy(0.5)

        next_value = strategy.get_next_value(current_activation, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, 0.622459, places=5)

        # Verify expected value after excitation starting at activation = MAX_ACTIVATION is
        # still approximately MAX_ACTIVATION
        strategy = SigmoidExciteStrategy()

        next_value = strategy.get_next_value(MAX_ACTIVATION, rate_in_hz=1.0)
        self.assertAlmostEqual(next_value, MAX_ACTIVATION, places=5)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=0.0)

        with self.assertRaises(ValueError):
            strategy.get_next_value(current_activation, rate_in_hz=-1.0)


class CognitiveContentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc', 'lidapy.LocalCommunicationProxy')

        lidapy.init(config=config)

    def test(self):
        c = CognitiveContent('value')
        c.activation = .75
        c.base_level_activation = .1
        c.incentive_salience = .2
        c.removal_threshold = .0001

        self.assertEqual(c.activation, .75)
        self.assertEqual(c.base_level_activation, .1)
        self.assertEqual(c.incentive_salience, .2)
        self.assertEqual(c.removal_threshold, .0001)

        # Test methods and attributes of wrapped class (str)
        self.assertEqual(c.capitalize(), 'Value')
        self.assertEqual(len(c), 5)
        self.assertEqual(c[0], 'v')


class FunctionsTestCases(unittest.TestCase):
    def test_create_class_instance(self):
        logger = create_class_instance('lidapy.ConsoleLogger')
        self.assertIsInstance(logger, lidapy.ConsoleLogger)

        # Test positional args
        logger = create_class_instance('lidapy.ConsoleLogger', lidapy.LOG_LEVEL_DEBUG)
        self.assertIsInstance(logger, lidapy.ConsoleLogger)
        self.assertEqual(logger.log_level, lidapy.LOG_LEVEL_DEBUG)

        # Test keyword args
        logger = create_class_instance('lidapy.ConsoleLogger', log_level=lidapy.LOG_LEVEL_DEBUG)
        self.assertIsInstance(logger, lidapy.ConsoleLogger)
        self.assertEqual(logger.log_level, lidapy.LOG_LEVEL_DEBUG)
