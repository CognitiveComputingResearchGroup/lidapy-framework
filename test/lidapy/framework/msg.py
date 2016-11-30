import unittest

from lidapy.framework.agent import AgentConfig
from lidapy.framework.msg import FrameworkTopic
from lidapy.framework.shared import FrameworkDependencyService, CognitiveContent
from lidapy.util.comm import LocalCommunicationProxy, MsgUtils, RosCommunicationProxy
from lidapy.util.logger import ConsoleLogger
from time import sleep

from lidapy.util.meta import Singleton
from std_msgs.msg import Float64


class LocalFrameworkTopicTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fds = FrameworkDependencyService()
        fds["ipc_proxy"] = LocalCommunicationProxy()
        fds["logger"] = ConsoleLogger()

        config = AgentConfig(config_file_override=True)
        config.set_global_param("rate_in_hz", 1000)

        fds["config"] = config

    def test_initializer(self):
        # Test initializer sets attributes correctly
        topic = FrameworkTopic(topic_name="my_topic",
                               msg_type=CognitiveContent)

        self.assertEqual(topic.topic_name, "my_topic", "Unexpected topic name")
        self.assertEqual(topic.msg_type, CognitiveContent, "Unexpected message type")

        # Test auto-generation of topic name
        topic = FrameworkTopic()

        self.assertIsNotNone(topic.topic_name, "Failed to auto-generate topic_name: topic_name is None")

    def test_create_pub_sub(self):
        topic = FrameworkTopic()

        pub = topic.create_publisher(max_queue_size=10, preprocessor=MsgUtils.serialize)
        self.assertIsNotNone(pub, "Publisher not returned from create_publisher")
        self.assertIsNotNone(topic.publisher, "Publisher not set as attribute in FrameworkTopic")
        self.assertEqual(pub, topic.publisher,
                         "Publisher returned from create_publisher does not match publisher set as attribute in FrameworkTopic")

        # Check publisher attributes
        self.assertEqual(pub.queue_size, 10, "Publisher's queue_size does not have expected value")
        self.assertEqual(pub.preprocessor, MsgUtils.serialize, "Publisher's preprocessor does not have expected value")

        sub = topic.create_subscriber(max_queue_size=10, postprocessor=MsgUtils.deserialize)
        self.assertIsNotNone(sub, "Subscriber not returned from create_subscriber")
        self.assertIsNotNone(topic.subscriber, "Subscriber not set as attribute in FrameworkTopic")
        self.assertEqual(sub, topic.subscriber,
                         "Subscriber returned from create_subscriber does not match subscriber set as attribute in FrameworkTopic")

        # Check subscriber attributes
        self.assertEqual(sub.queue_size, 10, "Subscriber's queue_size does not have expected value")
        self.assertEqual(sub.postprocessor, MsgUtils.deserialize,
                         "Subscriber's postprocessor does not have expected value")

    def test_pub_sub_consistency(self):
        topic = FrameworkTopic(msg_type=int)

        topic.create_publisher()
        topic.create_subscriber()

        self.assertEqual(topic.publisher.msg_type, topic.subscriber.msg_type,
                         "Message type mismatch between publisher and subscriber for topic")
        self.assertEqual(topic.publisher.topic_name, topic.subscriber.topic_name,
                         "Topic name mismatch between publisher and subscriber for topic")

    def test_publish(self):
        topic = FrameworkTopic()

        # Verify that create_publisher returns the publisher
        publisher = topic.create_publisher()

        # Verify that publish adds the expected message to the message queue
        msg = "test"
        publisher.publish(msg)

        self.assertEqual(len(publisher.msg_queue), 1, "Incorrect message length in message queue after publish")
        self.assertEqual(publisher.msg_queue.peek(), msg, "Unexpected message found in message queue")

        # Verify that publish raises an exception when None passed as message
        with self.assertRaises(Exception):
            publisher.publish(None)

    def test_publish_max_queue_size(self):
        topic = FrameworkTopic()

        publisher = topic.create_publisher(max_queue_size=10)
        self.assertEqual(publisher.queue_size, 10)

    def test_publish_preprocessor(self):
        topic = FrameworkTopic()

        topic.create_publisher(preprocessor=lambda m: m.upper())

        msg = "test"
        topic.publisher.publish(msg)

        self.assertEqual(len(topic.publisher.msg_queue), 1, "Incorrect message length in message queue after publish")
        self.assertEqual(topic.publisher.msg_queue.peek(), msg.upper(),
                         "Message failed to match expected content after preprocessing")

    def test_subscribe(self):
        topic = FrameworkTopic("my_topic")

        # Verify that create_subscriber returns the subscriber
        subscriber = topic.create_subscriber()
        self.assertIsNotNone(subscriber, "Subscriber not returned from create_subscriber")

        # Verify the subscriber attribute is set on topic
        self.assertIsNotNone(topic.subscriber, "Subscriber not set as attribute in FrameworkTopic")

        msg = "test"
        subscriber.msg_queue.push(msg)

        # Pause for subscriber thread to pull message from queue
        sleep(.01)

        self.assertEqual(len(subscriber.msg_queue), 0,
                         "Incorrect message length in message queue after subscriber invoked")

        msg_count = subscriber.msg_count
        self.assertEqual(msg_count, 1, "Incorrect number of pending messages in subscriber")

        next_msg = subscriber.get_next_msg()
        self.assertIsNotNone(next_msg, "get_next_msg incorrectly returned None when message available")
        self.assertEqual(next_msg, msg, "Unexpected message returned from subscriber")
        self.assertEqual(subscriber.msg_count, msg_count - 1, "Message count failed to decrement after get_next_msg")

        # Verify get_next_msg returns None if empty receive queue
        topic = FrameworkTopic()
        topic.create_subscriber()
        self.assertEqual(topic.subscriber.get_next_msg(), None,
                         "get_next_msg should have returned None when no message in receive queue")

        def test_subscribe_postprocessor(self):
            msg = "TEST"

            topic = FrameworkTopic()
            topic.create_subscriber(postprocessor=lambda m: m.lower())
            self.assertIsNotNone(topic.subscriber.postprocessor, "Postprocessor not set as attribute of subscriber")

            processed_msg = topic.subscriber.postprocessor(msg)
            self.assertEquals(processed_msg, msg.lower(), "Subscriber's postprocessor function not working as expected")

            topic.subscriber.msg_queue.push(msg)

            sleep(.01)

            self.assertEqual(len(topic.subscriber.msg_queue), 0,
                             "Subscriber failed to retrieve message from message queue")
            self.assertEqual(topic.subscriber.get_next_msg(), msg.lower(),
                             "Subscriber's postprocessor function not applied to received messsage")


# For this set of test cases the ROS master must be running
class RosFrameworkTopicTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        Singleton._remove(FrameworkDependencyService)

        fds = FrameworkDependencyService()
        fds["ipc_proxy"] = RosCommunicationProxy()
        fds["logger"] = ConsoleLogger()

        config = AgentConfig(config_file_override=True)
        config.set_global_param("rate_in_hz", 1000)

        fds["config"] = config

        fds["ipc_proxy"].initialize_node(name="test_node")

    def test_initializer(self):
        # Test initializer sets attributes correctly
        topic = FrameworkTopic(topic_name="my_topic",
                               msg_type=CognitiveContent)

        self.assertEqual(topic.topic_name, "my_topic", "Unexpected topic name")
        self.assertEqual(topic.msg_type, CognitiveContent, "Unexpected message type")

        # Test auto-generation of topic name
        topic = FrameworkTopic()

        self.assertIsNotNone(topic.topic_name, "Failed to auto-generate topic_name: topic_name is None")

    def test_create_pub_sub(self):
        topic = FrameworkTopic()

        pub = topic.create_publisher(max_queue_size=10, preprocessor=MsgUtils.serialize)
        self.assertIsNotNone(pub, "Publisher not returned from create_publisher")
        self.assertIsNotNone(topic.publisher, "Publisher not set as attribute in FrameworkTopic")
        self.assertEqual(pub, topic.publisher,
                         "Publisher returned from create_publisher does not match publisher set as attribute in FrameworkTopic")

        # Check publisher attributes
        self.assertEqual(pub.queue_size, 10, "Publisher's queue_size does not have expected value")
        self.assertEqual(pub.preprocessor, MsgUtils.serialize, "Publisher's preprocessor does not have expected value")

        sub = topic.create_subscriber(max_queue_size=10, postprocessor=MsgUtils.deserialize)
        self.assertIsNotNone(sub, "Subscriber not returned from create_subscriber")
        self.assertIsNotNone(topic.subscriber, "Subscriber not set as attribute in FrameworkTopic")
        self.assertEqual(sub, topic.subscriber,
                         "Subscriber returned from create_subscriber does not match subscriber set as attribute in FrameworkTopic")

        # Check subscriber attributes
        self.assertEqual(sub.queue_size, 10, "Subscriber's queue_size does not have expected value")
        self.assertEqual(sub.postprocessor, MsgUtils.deserialize,
                         "Subscriber's postprocessor does not have expected value")

    def test_pub_sub_consistency(self):
        topic = FrameworkTopic(msg_type=Float64)

        topic.create_publisher()
        topic.create_subscriber()

        self.assertEqual(topic.publisher.msg_type, topic.subscriber.msg_type,
                         "Message type mismatch between publisher and subscriber for topic")
        self.assertEqual(topic.publisher.topic_name, topic.subscriber.topic_name,
                         "Topic name mismatch between publisher and subscriber for topic")

    def test_publish(self):
        topic = FrameworkTopic()

        # Verify that create_publisher returns the publisher
        publisher = topic.create_publisher()

        # Verify that publish adds the expected message to the message queue
        msg = "test"
        publisher.publish(msg)

        # Verify that publish raises an exception when None passed as message
        with self.assertRaises(Exception):
            publisher.publish(None)

    def test_publish_max_queue_size(self):
        topic = FrameworkTopic()

        publisher = topic.create_publisher(max_queue_size=10)
        self.assertEqual(publisher.queue_size, 10)

    def test_publish_preprocessor(self):
        topic = FrameworkTopic()

        topic.create_publisher(preprocessor=lambda m: m.upper())

        msg = "test"
        topic.publisher.publish(msg)

    def test_subscribe(self):
        topic = FrameworkTopic("my_topic")

        # Verify that create_subscriber returns the subscriber
        subscriber = topic.create_subscriber()
        self.assertIsNotNone(subscriber, "Subscriber not returned from create_subscriber")

        # Verify the subscriber attribute is set on topic
        self.assertIsNotNone(topic.subscriber, "Subscriber not set as attribute in FrameworkTopic")

        # Verify get_next_msg returns None if empty receive queue
        topic = FrameworkTopic()
        topic.create_subscriber()
        self.assertEqual(topic.subscriber.get_next_msg(), None,
                         "get_next_msg should have returned None when no message in receive queue")

    def test_subscribe_postprocessor(self):
        msg = "TEST"

        topic = FrameworkTopic()
        topic.create_subscriber(postprocessor=lambda m: m.lower())
        self.assertIsNotNone(topic.subscriber.postprocessor, "Postprocessor not set as attribute of subscriber")

        processed_msg = topic.subscriber.postprocessor(msg)
        self.assertEquals(processed_msg, msg.lower(), "Subscriber's postprocessor function not working as expected")
