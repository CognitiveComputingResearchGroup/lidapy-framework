#!/usr/bin/env python
'''
Created on Apr 20, 2016

@author: Sean Kugele
'''
from lidapy.framework.module import FrameworkModule


class ProceduralMemoryModule(FrameworkModule):

    def __init__(self):
        FrameworkModule.__init__(self, "ProceduralMemoryModule")
        return



if __name__ == '__main__':

    try:
        module = ProceduralMemoryModule()
        module.run(10)

    except Exception as e:
        print e

    finally:
        pass

