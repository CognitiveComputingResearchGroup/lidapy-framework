#!/usr/bin/env python
'''
Created on May 6, 2016

@author: Tamas Madl
'''
from lidapy.module.sensory_motor_memory import SensoryMotorMemoryModule
from lidapy.framework.msg import Behavior, ConsciousContent
from lidapy.util.rosutils import StateSetter
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from lidapy.util import logger
import time

class AtlasSensoryMotorMemoryModule(SensoryMotorMemoryModule):
    def __init__(self, fakewalking=True):
        super(SensoryMotorMemoryModule, self).__init__("AtlasSensoryMotorMemoryModule")
        
        # initialize StateSetter, allowing Sensory-Motor Memory to control individual joints of the Atlas
        self.state_setter = StateSetter()

        if fakewalking:
            # initialize fake walking
            # TODO: put into config file instead
            logger.info("Using fake walking...")
            self._publishers["/atlas/mode"].publish("pid_stand")
            self._publishers["/atlas/mode"].publish("pinned")
            time.sleep(1)
            
            logger.info("Resetting pose (looking straight ahead)...")
            # first call fails sometimes, not sure why
            self.state_setter.setstate({'atlas::neck_ay': 0})
            time.sleep(1)
            self.state_setter.setstate({'atlas::neck_ay': 0})        
        

    def _add_publisher(self, topic, msg_type, queue_size=0):
        super(AtlasSensoryMotorMemoryModule, self)._add_publisher(topic, msg_type, queue_size)
        
    def _add_subscriber(self, topic, msg_type, callback=None, callback_args={}):
        super(AtlasSensoryMotorMemoryModule, self)._add_subscriber(topic, msg_type, callback, callback_args)

    def add_publishers(self, queue_size=10):
        """
        In order to control a Boston Dynamics Atlas robot,
        this method subscribes to 
            1) /atlas/mode (to set up "fake walking", see http://gazebosim.org/tutorials?tut=drcsim_fakewalking&cat=drcsim)
            2) /atlas/cmd_vel (to actually move the robot)
        """
        
        pubs = [
                {"topic": "/atlas/mode", "msg_type": String},
                {"topic": "/atlas/cmd_vel", "msg_type": Twist}
        ]
        for pub in pubs:
            self._add_publisher(pub["topic"], pub["msg_type"], queue_size=queue_size)
            
    def add_subscribers(self):
        subs = [{"topic": "/lida/selected_behaviors", "msg_type": Behavior.msg_type(), "callback": self.execute_behavior},
                {"topic": "/lida/global_broadcast", "msg_type": ConsciousContent.msg_type(), "callback": None}]
        for sub in subs:
            self._add_subscriber(sub["topic"], sub["msg_type"], sub["callback"])
            
    def execute_behavior(self, msg):
        """
        Execute the behavior selected by ActionSelection (arriving through /lida/selected_behaviors)
        """
        print msg

if __name__ == '__main__':
    try:
        module = AtlasSensoryMotorMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
