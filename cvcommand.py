# -*- coding: utf-8 -*-
"""
Created on Tue May 02 17:08:35 2017

@author: yfz
"""

import CVController

capture = CVController()

val = raw_input("Insert Valve Type: ")

(offset, angle, orientation) = capture.processCommand(val)


