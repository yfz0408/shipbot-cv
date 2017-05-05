# -*- coding: utf-8 -*-
"""
Created on Tue May 02 17:08:35 2017

@author: yfz
"""

import CVController

capture = CVController.CVController()

(offset, angle, orientation) = capture.processCommand("V1")

print offset
print angle
print orientation
