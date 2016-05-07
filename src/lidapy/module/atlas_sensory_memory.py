#!/usr/bin/env python
'''
Created on May 6, 2016

@author: Tamas Madl
'''
from lidapy.framework.module import FrameworkModule
from lidapy.framework.msg import Feature
from random import randint

# TODO: Replace this with LIDA messages
from std_msgs.msg import String
from lidapy.module.sensory_memory import SensoryMemoryModule

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from threading import Thread
from lidapy.util.rosutils import SingleSubscription
import matplotlib.image as mpimg
import time, numpy
from lidapy.util import logger
import matplotlib.image as mpimg
from lidapy.framework.agent import AgentConfig

class AtlasSensoryMemoryModule(SensoryMemoryModule):
    """
        Subscribe to Atlas' sensory ROS topics, and cache data to disk
    """
    
    def __init__(self):
        self.module_name = "AtlasSensoryMemoryModule"
        self._publishers = {}
        self._received_msgs = {}
        self._config = AgentConfig()

        self._threads = []
        self._topic_data = {}
        self.running = True        
        # TODO put into config
        self.ISIZE = 800 

        self.add_publishers()
        self.add_subscribers()

    def add_subscribers(self):
        subs = [
                #{"topic": "/lida/environment", "msg_type": String}
                {"topic": "/multisense_sl/camera/left/image_color/compressed", "msg_type": CompressedImage, "cache_path": "../../../sensor_data_cache/snapshot.jpg"},
                {"topic": "/multisense_sl/points2", "msg_type": PointCloud2, "cache_path": "../../../sensor_data_cache/depth.dat"}
        ]
        for i in range(len(subs)):
            p = Thread(target=self.subscribe, args=(subs[i]["topic"], subs[i]["msg_type"], subs[i]["cache_path"]))
            p.start()
            self._threads.append(p)

    def subscribe(self, topic, msgtype, savepath=None, sleep=0.2):
        """
        Subscribe to the ROS topic and save data to disk
        
        Parameters
        ----------
        topic: str
            topic to subscribe to
        savepath: str
            path to save data to
        msgtype: object (CompressedImage OR PointCloud2 OR Odometry)
            data type to expect
        sleep: float
            waiting time before reading and storing sensory data anew
        """
        logger.info("subscribing to "+topic+" (running: "+str(self.running)+")")
        while self.running:
            sub = SingleSubscription()
            sub.snapshot(topic, savepath, msgtype, True)
            sub.waituntildone()
            if msgtype == CompressedImage:
                try:
                    # TODO convert from CompressedImage to numpy array without 
                    self._topic_data[topic] = mpimg.imread(savepath)
                except:
                    logger.info("no image found at {}".format(savepath))
            elif msgtype == PointCloud2:
                (points, notnanindices, N) = sub.data
                
                # calc depth map
                depthmap = numpy.zeros((self.ISIZE*self.ISIZE,))
                if numpy.any(notnanindices):
                    depthmap[notnanindices] = numpy.sqrt(numpy.sum(numpy.square(points), axis = 1))
                    self._topic_data[topic] = numpy.reshape(depthmap, (self.ISIZE, self.ISIZE))
            else:
                self._topic_data[topic] = sub.data
            time.sleep(sleep)

    def advance(self):
        msg = Feature()
        msg.id = str(randint(0, 1e15 - 1))
        super(SensoryMemoryModule, self).publish("/lida/detected_features", msg)

if __name__ == '__main__':

    try:
        module = AtlasSensoryMemoryModule()
        module.run()

    except Exception as e:
        print e

    finally:
        pass
