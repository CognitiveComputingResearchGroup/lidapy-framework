#!/usr/bin/env python
'''
Created on Apr 21, 2016

@author: Sean Kugele
'''
from rospy import logdebug, loginfo, logwarn, logerr, logfatal


def debug(msg):
    logdebug(msg)


def info(msg):
    loginfo(msg)


def warn(msg):
    logwarn(msg)


def error(msg):
    logerr(msg)


def fatal(msg):
    logfatal(msg)

