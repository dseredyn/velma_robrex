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

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

class TfPub:

    def processFeedback(self, feedback):
#        print "feedback", feedback.marker_name, feedback.control_name, feedback.event_type
        if ( feedback.marker_name == 'jar_marker' ): # and ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK ) and ( feedback.control_name == "button" ):
            self.T_W_J = pm.fromMsg(feedback.pose)

    def insert6DofGlobalMarker(self):
        int_position_marker = InteractiveMarker()
        int_position_marker.header.frame_id = 'world'
        int_position_marker.name = 'jar_marker'
        int_position_marker.scale = 0.2
        int_position_marker.pose = pm.toMsg(self.T_W_J)

        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));

        box = self.createAxisMarkerControl(Point(0.15,0.015,0.015), Point(0.0, 0.0, 0.0) )
        box.interaction_mode = InteractiveMarkerControl.BUTTON
        box.name = 'button'
        int_position_marker.controls.append( box )
        self.server.insert(int_position_marker, self.processFeedback);
        self.server.applyChanges();

    def createAxisMarkerControl(self, scale, position):
        markerX = Marker()
        markerX.type = Marker.ARROW
        markerX.scale = scale
        markerX.pose.position = position
        ori = quaternion_about_axis(0, [0, 1 ,0])
        markerX.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerX.color = ColorRGBA(1,0,0,1)
        markerY = Marker()
        markerY.type = Marker.ARROW
        markerY.scale = scale
        markerY.pose.position = position
        ori = quaternion_about_axis(math.pi/2.0, [0, 0 ,1])
        markerY.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerY.color = ColorRGBA(0,1,0,1)
        markerZ = Marker()
        markerZ.type = Marker.ARROW
        markerZ.scale = scale
        markerZ.pose.position = position
        ori = quaternion_about_axis(-math.pi/2.0, [0, 1 ,0])
        markerZ.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerZ.color = ColorRGBA(0,0,1,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( markerX );
        control.markers.append( markerY );
        control.markers.append( markerZ );
        return control

    def createInteractiveMarkerControl6DOF(self, mode, axis):
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        if mode == InteractiveMarkerControl.ROTATE_AXIS:
            control.name = 'rotate_';
        if mode == InteractiveMarkerControl.MOVE_AXIS:
            control.name = 'move_';
        if axis == 'x':
            control.orientation = Quaternion(1,0,0,1)
            control.name = control.name+'x';
        if axis == 'y':
            control.orientation = Quaternion(0,1,0,1)
            control.name = control.name+'x';
        if axis == 'z':
            control.orientation = Quaternion(0,0,1,1)
            control.name = control.name+'x';
        control.interaction_mode = mode
        return control

    def spin(self):
        rospy.sleep(1.0)
        while not rospy.is_shutdown():
            q = self.T_W_J.M.GetQuaternion()
            self.br.sendTransform([self.T_W_J.p.x(), self.T_W_J.p.y(), self.T_W_J.p.z()], [q[0], q[1], q[2], q[3]], rospy.Time.now(), "jar", "world")
            rospy.sleep(0.1)

    def __init__(self):
        self.T_W_J = PyKDL.Frame(PyKDL.Vector(1,0,1))
        self.br = tf.TransformBroadcaster()

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer('interactive_tf_pub')
        self.insert6DofGlobalMarker()
        self.server.applyChanges();

if __name__ == "__main__":
    rospy.init_node('interactive_tf_pub', anonymous=True)

    tp = TfPub()
    tp.spin()

