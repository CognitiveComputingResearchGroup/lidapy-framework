'''
Created on May 6, 2016

@author: Tamas Madl
'''

from lidapy.framework.agent import AgentConfig
from lidapy.module.atlas_sensory_motor_memory import AtlasSensoryMotorMemoryModule
from lidapy.framework.msg import Behavior
from lidapy.util.comm import get_publisher
from lidapy.util import logger
import time, traceback, random
from lidapy.module.atlas_sensory_memory import AtlasSensoryMemoryModule

plt = None
try:
    import matplotlib.pyplot as plt
    plt.ion() # interactive (non-blocking) plotting
except:
    print "importing matplotlib failed - take a look at the contents of 'sensor_data_cache' for sensory memory snapshots"

class AgentStarter(object):
    def __init__(self, config_filepath=None, fakewalking=True):
        """
        Hackish initialization of an agent - for testing purposes only
        """
        try:
            self.config = AgentConfig(config_filepath)
        except:
            traceback.print_exc()
        
        # TODO: load from agent definition XML (factory pattern)
        self.modules = {}
        self.modules["SensoryMotorMemory"] = AtlasSensoryMotorMemoryModule()
        self.modules["SensoryMemory"] = AtlasSensoryMemoryModule()
        
        # TODO: proper cognitive cycle instead
        behavior_publisher = get_publisher("/lida/selected_behaviors", Behavior, queue_size=10)
        for i in range(100):
            linear_speed = random.random()*2
            angular_speed = random.random()*0.5
            logger.info("Executing random movement: {}".format(linear_speed)+", {}".format(angular_speed))
            self.modules["SensoryMotorMemory"].move(linear_speed, angular_speed)
            
            behavior = Behavior()
            behavior.data = i
            logger.info("Faking selection of action: {}".format(behavior))
            behavior_publisher.publish(behavior)
            time.sleep(1)
            
            if plt != None:
                topics = self.modules["SensoryMemory"]._topic_data.keys()
                for i in range(len(topics)):
                    plt.subplot(1,len(topics),i+1)
                    plt.imshow(self.modules["SensoryMemory"]._topic_data[topics[i]])
                plt.pause(0.0001)  
        plt.show(block=True) # blocking plot of last sensory images 
            
if __name__ == "__main__":
    starter = AgentStarter()
