import unittest
import mock

from wander import get_next_cmd, FORWARD, STOP, wander


class TestWander(unittest.TestCase):
    @mock.patch(target='wander.rospy')
    def test(self, mock_rospy):
        mock_rospy.Time.now.return_value = 100

        self.assertEqual(STOP.linear.x, 0.0,
                         'STOP.linear.x does not match expected value')
        self.assertEqual(FORWARD.linear.x, 0.5,
                         'FORWARD.linear.x does not match expected value')

        self.assertEqual(get_next_cmd(current_cmd=STOP, switch_time=150, inc=10), (STOP, 150))
        self.assertEqual(get_next_cmd(current_cmd=FORWARD, switch_time=150, inc=10), (FORWARD, 150))
        self.assertEqual(get_next_cmd(current_cmd=FORWARD, switch_time=100, inc=10), (STOP, 110))
        self.assertEqual(get_next_cmd(current_cmd=STOP, switch_time=100, inc=10), (FORWARD, 110))

    @mock.patch(target='wander.rospy')
    def test_wander(self, mock_rospy):
        mock_rospy.init_node.return_value = None
        mock_rospy.is_shutdown.return_value = True

        wander()
        self.assertTrue(mock_rospy.init_node.called, 'init_node not called')
        self.assertTrue(mock_rospy.is_shutdown.called, 'is_shutdown not called')
