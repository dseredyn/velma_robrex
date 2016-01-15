#!/usr/bin/env python

# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib
roslib.load_manifest('velma_robrex')

import rospy
import sensor_msgs.msg
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
import object_recognition_msgs.msg

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

class TfPub:

    def spin(self):
        rospy.sleep(1.0)
        while not rospy.is_shutdown():
            if self.frame_id != None:
                q = self.T_C_Jb.M.GetQuaternion()
                self.br.sendTransform([self.T_C_Jb.p.x(), self.T_C_Jb.p.y(), self.T_C_Jb.p.z()], [q[0], q[1], q[2], q[3]], rospy.Time.now(), "jar_base", self.frame_id)
                q = self.T_C_J.M.GetQuaternion()
                self.br.sendTransform([self.T_C_J.p.x(), self.T_C_J.p.y(), self.T_C_J.p.z()], [q[0], q[1], q[2], q[3]], rospy.Time.now(), "jar", self.frame_id)
            rospy.sleep(0.1)

    def recognizedCallback(self, msg):
        self.T_C_Jb = pm.fromMsg(msg.pose.pose.pose)
        self.T_C_J = self.T_C_Jb * PyKDL.Frame(PyKDL.Vector(0,0,0.11))
        self.frame_id = msg.header.frame_id

    def __init__(self):
        self.T_C_Jb = PyKDL.Frame()
        self.T_C_J = PyKDL.Frame()
        self.frame_id = None
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('/recognized_objects', object_recognition_msgs.msg.RecognizedObject, self.recognizedCallback)

if __name__ == "__main__":
    rospy.init_node('recognized_tf_pub', anonymous=True)

    tp = TfPub()
    tp.spin()

