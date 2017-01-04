import unittest

from lidapy import CognitiveContent
from lidapy import MsgUtils
from lidapy import RosMsgUtils
from std_msgs.msg import String


class MsgUtilsTest(unittest.TestCase):
    def test_serializer(self):
        obj = CognitiveContent("value")

        s_obj = MsgUtils.serialize(obj)
        self.assertTrue(isinstance(s_obj, basestring))

        d_obj = MsgUtils.deserialize(s_obj)
        self.assertTrue(isinstance(d_obj, CognitiveContent))
        self.assertEqual(d_obj, obj)


class RosMsgUtilsTest(unittest.TestCase):
    def test_msg_wrapper(self):
        obj = MsgUtils.serialize(CognitiveContent("value"))

        w_obj = RosMsgUtils.wrap(obj, String, "data")

        self.assertTrue(isinstance(w_obj, String))
        self.assertTrue(hasattr(w_obj, "data"))
        self.assertEqual(w_obj.data, obj)

        uw_obj = RosMsgUtils.unwrap(w_obj, "data")

        self.assertTrue(isinstance(MsgUtils.deserialize(uw_obj), CognitiveContent))
