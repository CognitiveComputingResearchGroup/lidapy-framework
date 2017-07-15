#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

STOP = Twist()
STOP.linear.x = 0.0

FORWARD = Twist()
FORWARD.linear.x = 0.5


def get_next_cmd(current_cmd, switch_time, inc=10):
    if not rospy.Time.now() >= switch_time:
        return current_cmd, switch_time

    else:
        if current_cmd == FORWARD:
            return STOP, rospy.Time.now() + rospy.Duration(inc)
        else:
            return FORWARD, rospy.Time.now() + rospy.Duration(inc)


def wander():
    pub = rospy.Publisher(name='cmd_vel', data_class=Twist, queue_size=1)

    cmd = STOP
    switch_time = rospy.Time.now()
    while not rospy.is_shutdown():
        cmd, switch_time = get_next_cmd(current_cmd=cmd,
                                        switch_time=switch_time)
        pub.publish(cmd)
        rospy.Rate(1).sleep()


rospy.init_node('wanderbot')
wander()
