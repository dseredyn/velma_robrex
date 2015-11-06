// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#ifndef BARRETT_HAND_INTERFACE_H__
#define BARRETT_HAND_INTERFACE_H__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <barrett_hand_controller_msgs/Empty.h>
#include <barrett_hand_controller_msgs/BHMoveAction.h>

#include "Eigen/Dense"

class BarrettHandInterface {
protected:
    ros::NodeHandle nh_;
    const std::string prefix_;
    actionlib::SimpleActionClient<barrett_hand_controller_msgs::BHMoveAction> action_move_;
    ros::Publisher reset_fingers_pub_;
    ros::Publisher calibrate_sensors_pub_;

public:
    BarrettHandInterface(const std::string &prefix);
    ~BarrettHandInterface();
    void resetFingers();
    void calibrateTactileSensors();

    // spread, f1, f2, f3
    void moveFingers(const Eigen::Vector4d &q, const Eigen::Vector4d &v, const Eigen::Vector4d &t, double max_pressure, bool hold);
    bool waitForSuccess(double max_duration);
};

#endif  // BARRETT_HAND_INTERFACE_H__

