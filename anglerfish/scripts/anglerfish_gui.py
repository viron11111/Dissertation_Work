#!/usr/bin/python

import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import python_qt_binding.QtGui as qtg
import time


class Anglerfish_gui(Plugin):

    def __init__(self, context):
        super(Anglerfish_gui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Anglerfish_gui')

