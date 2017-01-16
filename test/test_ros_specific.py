###############################################################################
## Unit tests Of ROS Specific Functionality                                   #
##                                                                            #
## NOTE: The RosMaster process must be running for these test cases to pass.  #
###############################################################################

import collections
import unittest

import lidapy
from lidapy import CognitiveContent
from lidapy import Config
from lidapy import MsgUtils
from lidapy import RosMsgUtils
from lidapy import Topic
from std_msgs.msg import String


class RosTopicTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc_proxy', 'lidapy.RosCommunicationProxy')

        lidapy.init(process_name='test')

    def test_create_pub_sub(self):
        topic = Topic(name='topic')
        topic._init_publisher()
        topic._init_subscriber()

        self.assertEqual(type(topic._publisher), lidapy.RosTopicPublisher)
        self.assertEqual(type(topic._subscriber), lidapy.RosTopicSubscriber)

        self.assertEqual(topic._publisher.msg_type, topic._subscriber.msg_type)

    def test(self):
        topic = Topic(name='RosTopicTest_Topic')

        sent_msg = '1'

        recv_msg = None
        while not recv_msg:
            topic.send(sent_msg)
            recv_msg = topic.receive(timeout=0.001)

        self.assertEqual(sent_msg, recv_msg)

    def test_preprocessor(self):
        topic = Topic('topic_2', preprocessor=lambda x: str(x) + '_processed', postprocessor=lambda x: x.data)

        sent_msg = '1'

        recv_msg = None
        while not recv_msg:
            topic.send(sent_msg)
            recv_msg = topic.receive(timeout=0.001)

        if '_processed' not in recv_msg:
            self.fail('Failed to find expected string')

    def test_postprocessor(self):
        topic = Topic('topic_3', preprocessor=lambda x: x, postprocessor=lambda x: x.data.replace('_processed', ''))

        recv_msg = None
        while not recv_msg:
            topic.send('1_processed')
            recv_msg = topic.receive(timeout=0.001)

        self.assertEqual('1', recv_msg)


class RosCommunicationProxyTest(unittest.TestCase):
    # Access to internal LidaPy globals for testing low-level operations
    _var = None

    def setUp(self):
        config = Config()
        config.set_param('logger', 'lidapy.ConsoleLogger')
        config.set_param('ipc_proxy', 'lidapy.RosCommunicationProxy')

        self._var = lidapy.init(process_name='test')

    def test_publisher(self):
        def callback(msg):
            pass

        pub = self._var.ipc.get_publisher(topic_name='topic', preprocessor=callback)

        self.assertIsInstance(pub.__class__, lidapy.RosTopicPublisher.__class__)
        self.assertEqual(pub.topic_name, 'topic')
        self.assertEqual(pub.preprocessor, callback)

    def test_subscriber(self):
        def callback(msg):
            pass

        observers = collections.deque()
        sub = self._var.ipc.get_subscriber(topic_name='topic', postprocessor=callback)

        self.assertIsInstance(sub.__class__, lidapy.RosTopicSubscriber.__class__)
        self.assertEqual(sub.topic_name, 'topic')
        self.assertEqual(sub.postprocessor, callback)

    def test_service(self):
        # Not implemented
        pass

    def test_service_proxy(self):
        # Not implemented
        pass


class RosMsgUtilsTest(unittest.TestCase):
    def test_msg_wrapper(self):
        cc = CognitiveContent('value')
        cc.activation = .75
        cc.base_level_activation = .5
        cc.incentive_salience = .25
        cc.removal_threshold = .1

        obj = MsgUtils.serialize(cc)

        w_obj = RosMsgUtils.wrap(obj, String, 'data')

        self.assertTrue(isinstance(w_obj, String))
        self.assertEqual(w_obj.data, obj)

        uw_obj = RosMsgUtils.unwrap(w_obj, 'data')
        cc_new = MsgUtils.deserialize(uw_obj)

        self.assertIsInstance(cc_new, CognitiveContent)
        self.assertEqual(cc_new.activation, cc.activation)
        self.assertEqual(cc_new.base_level_activation, cc.base_level_activation)
        self.assertEqual(cc_new.incentive_salience, cc.incentive_salience)
        self.assertEqual(cc_new.removal_threshold, cc.removal_threshold)
        self.assertEqual(cc_new._value, cc._value)

# class ParameterServiceTest(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         lidapy.init(process_name='test')
#
#     def test(self):
#         ps = ParameterService()
#
#         ps.set_param(name='new_param', value='expected_value', section='globals')
#         actual_value = ps.get_param(name='new_param', section='globals')
#         self.assertEqual('expected_value', actual_value)
