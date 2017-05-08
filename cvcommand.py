# -*- coding: utf-8 -*-
"""
Created on Tue May 02 17:08:35 2017

@author: yfz
"""

import CVController

capture = CVController.CVController()

device = raw_input("type in device:")

(offset, angle, orientation) = capture.processCommand(device)

print offset
print angle
print orientation
