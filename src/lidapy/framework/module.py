'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
class FrameworkModule:

        
    def __init__(self, name):
        self.name = name
        
        self._publishers = {}
        self._subscribers = {}
        
        return
    
    def register(self):
        return
    
    def addPublisher(self, topic, msg_type, queue_size=0):
        return
    
    def addSubscriber(self, topic, msg_type, callback, args=[]):
        return