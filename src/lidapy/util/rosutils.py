'''
Created on May 6, 2016

@author: Tamas Madl
'''

import rospy
import os
import time
import cPickle as pickle
import numpy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Vector3

IMGTOPIC = "/multisense_sl/camera/left/image_color/compressed"
DEPTHTOPIC = "/multisense_sl/points2"

class DVector3(Vector3):
    defaultcoord = 0
    def __init__(self, coords=[0,0,0], defaultcoord=0):
        self.set(coords)
        self.defaultcoord = defaultcoord
    def set(self, coords=[0,0,0]):
        if type(coords) == list:
            [self.x, self.y, self.z] = coords
        else:
            if self.defaultcoord == 0: 
                self.x = coords
            elif self.defaultcoord == 1: 
                self.y = coords
            elif self.defaultcoord == 2: 
                self.z = coords

class SingleSubscription(object):
    """
    Utility class for obtaining (and saving to disk) a single image from a ROS topic
    """
    
    def snapshot(self, rospath, dest_path, msg_type = CompressedImage, save_to_file=True):    
        """
        Subscribe to the ROS topic and prepare to
        save the incoming image to disk in the dest_dir directory
        """
        self.topic = rospath
        self.save_to_file = save_to_file
        # flag to indicate when the image has been saved
        self.done = False          
        self.path = os.path.abspath(dest_path) 
        self.msg_type = msg_type
        # subscribe to ROS topic        
        self.subscriber = rospy.Subscriber(rospath, msg_type, self.callback)

    def callback(self, ros_msg):
        """ 
        This method is invoked each time a new ROS message is generated.
        the message is of type CompressedImage (camera) or PointCloud2 (depth)
        """
        self.msg = ros_msg
        
        if self.msg_type == PointCloud2:
            # preprocess to get rid of NaN points (pixels of unknown distance)
            rawpoints = numpy.array(list(point_cloud2.read_points(ros_msg, skip_nans=False)), dtype=numpy.float32)[:, :3]
            notnanindices = ~numpy.isnan(rawpoints[:, 0])
            self.data = (rawpoints[notnanindices], notnanindices, len(rawpoints))
        elif self.msg_type == CompressedImage:
            self.data = ros_msg.data
        else:
            self.data = ros_msg
        
        # we don't need to be called again
        self.subscriber.unregister()
        
        if self.save_to_file:
            # create directories if necessary
            dest_dir = os.path.split(self.path)[0]    
            if not os.path.exists(dest_dir):
                os.makedirs(dest_dir)
            # write data to disk
            if self.msg_type == CompressedImage:
                f = open(self.path, 'w')
                f.write(self.data)
                f.close()
            else:
                f = open(self.path, 'wb')
                pickle.dump(self.data, f)
                f.close()
        
        self.done = True
        
    def waituntildone(self, timeout=30):
        for i in range(timeout*20):
            if self.done: 
                return True
            time.sleep(0.05)
        print "TIMEOUT while listening to ",self.topic,"!"
        return False
        
        
############## world state utils 

import sys
import StringIO
from rosservice import _rosservice_cmd_call

from atlas_msgs.msg import AtlasCommand
from sensor_msgs.msg import JointState

class StateListener(object):
    """
    Obtain joint states from Atlas
    """
    
    def __init__(self):
        self.currentJointState = None
    
    def jointcallback(self, data):
        #if self.currentJointState == None: print data
        self.currentJointState = data
    
    def getstate(self, listeningtime=0.2):
        subscriber = rospy.Subscriber("/atlas/joint_states", JointState, self.jointcallback)
        rospy.rostime.wallsleep(listeningtime)
        subscriber.unregister()
        return self.currentJointState
        
class StateSetter(object):
    """
    Set joint states on Atlas robot
    """
    
    atlasJointNames = [
      'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
      'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
      'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
      'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
      'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']
        
    NUM_JOINTS = len(atlasJointNames)
        
    def __init__(self):
        self.ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand, queue_size=1)
    
    def setstate(self, jointvalues={}, waitingtime=0.2):
        """Set the states of all joints of the Atlas robot

        Parameters
        ----------
        jointvalues : dict
            A dictionary of of joint values, specifying joint names and their desired values. It can specify one, multiple or all joints.
            Example (standing position):
            {
                 'atlas::back_lbz': 0.0,
                 'atlas::back_mby': 0.002,
                 'atlas::back_ubx': 0.0,
                 'atlas::l_arm_elx': 0.498,
                 'atlas::l_arm_ely': 2.001,
                 'atlas::l_arm_mwx': -0.004,
                 'atlas::l_arm_shx': -1.303,
                 'atlas::l_arm_usy': 0.3,
                 'atlas::l_arm_uwy': 0.0,
                 'atlas::l_leg_kny': 0.518,
                 'atlas::l_leg_lax': -0.062,
                 'atlas::l_leg_lhy': -0.233,
                 'atlas::l_leg_mhx': 0.062,
                 'atlas::l_leg_uay': -0.276,
                 'atlas::l_leg_uhz': -0.0,
                 'atlas::neck_ay': -0.001,
                 'atlas::r_arm_elx': -0.498,
                 'atlas::r_arm_ely': 2.001,
                 'atlas::r_arm_mwx': 0.004,
                 'atlas::r_arm_shx': 1.303,
                 'atlas::r_arm_usy': 0.3,
                 'atlas::r_arm_uwy': 0.0,
                 'atlas::r_leg_kny': 0.518,
                 'atlas::r_leg_lax': 0.062,
                 'atlas::r_leg_lhy': -0.233,
                 'atlas::r_leg_mhx': -0.062,
                 'atlas::r_leg_uay': -0.276,
                 'atlas::r_leg_uhz': 0.0
            } 

        waitingtime : float
            number of seconds given to ROS to assume the specified joint positions before continuing

        Returns
        -------
        self : returns an instance of self.
        """
        
        try:
            if type(jointvalues)==list:
                newposition = jointvalues
            else:
                newposition = list(StateListener().getstate().position) # get current state
                for k in jointvalues.keys():
                    newposition[StateSetter.atlasJointNames.index(k)] = jointvalues[k]
                
            stand_prep_msg = AtlasCommand()
            stand_prep_msg.header.stamp = rospy.Time.now()
            stand_prep_msg.position = newposition
            stand_prep_msg.velocity = [0.0] * StateSetter.NUM_JOINTS
            stand_prep_msg.effort = [0.0] * StateSetter.NUM_JOINTS
            stand_prep_msg.k_effort = [255] * StateSetter.NUM_JOINTS
    
            self.ac_pub.publish(stand_prep_msg)
            rospy.rostime.wallsleep(waitingtime)
        except Exception,e:
            print "FAILED TO SET ATLAS JOINTS. ", e

def getobjpose(modelname='atlas'):
    """
    Get pose (ground truth position and orientation) of ANY object in the environment (only use for debugging!)
    """
    
    stdout = sys.stdout
    sys.stdout = StringIO.StringIO() # replace by stringio to capture model command
    _rosservice_cmd_call([1,2,'gazebo/get_model_state', '{model_name: '+modelname+'}'])
    sys.stdout.seek(0)
    outstr = sys.stdout.read()
    sys.stdout = stdout # restore sysout
    
    parsevars = ["x", "y", "z"]
    pos = []
    for v in parsevars:
        s = outstr[(outstr.index(v+": ")+3):(outstr.index(v+": ")+14)]
        s = s.split(' ')[0].strip()
        pos.append(float(s))
    outstr = outstr[outstr.index('orientation:'):]
    parsevars = ["x", "y", "z", "w"]
    angle = []
    for v in parsevars:
        s = outstr[(outstr.index(v+": ")+3):(outstr.index(v+": ")+14)]
        s = s.split(' ')[0].strip()
        angle.append(float(s))
    return pos, angle

def setobjpose(modelname='atlas', pos=[0,0,0], angle=[0,0,0,0], twist=[0,0,0,0,0,0]):
    """
    Set ground truth pose of any object in the environment
    Experimental!!
    """
    print "TELEPORTING ",modelname," TO ",pos,angle
    args = "{model_state: { model_name: %s, pose: { position: { x: %f, y: %f, z: %f }, orientation: {x: %f, y: %f, z: %f, w: %f } }, twist: { linear: {x: %f , y: %f ,z: %f } , angular: { x: %f , y: %f, z: %f} } , reference_frame: world } }" % (modelname, pos[0], pos[1], pos[2], angle[0], angle[1], angle[2], angle[3], twist[0], twist[1], twist[2], twist[3], twist[4], twist[5])
    _rosservice_cmd_call(["-","-",'gazebo/set_model_state', args])