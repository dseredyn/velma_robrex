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

#include "barrett_hand_interface.h"

    BarrettHandInterface::BarrettHandInterface(const std::string &prefix) :
        nh_(),
        prefix_(prefix),
        action_move_(std::string("/") + prefix_ + "_hand/move_hand", true)
    {
    }

    BarrettHandInterface::~BarrettHandInterface() {
    }

    void BarrettHandInterface::resetFingers() {
        barrett_hand_controller_msgs::Empty reset_fingers;
        if (!ros::service::call(std::string("/") + prefix_ + "_hand/reset_fingers", reset_fingers)) {
            std::cout << "ERROR: ros::service::call(\"/" << prefix_ << "_hand/reset_fingers\" " << std::endl;
        }
    }

    void BarrettHandInterface::moveFingers(const Eigen::Vector4d &q, const Eigen::Vector4d &v, const Eigen::Vector4d &t, double max_pressure, bool hold) {
        action_move_.waitForServer();
        barrett_hand_controller_msgs::BHMoveGoal goal;
        goal.name.push_back(prefix_+"_HandFingerOneKnuckleOneJoint");
        goal.name.push_back(prefix_+"_HandFingerOneKnuckleTwoJoint");
        goal.name.push_back(prefix_+"_HandFingerTwoKnuckleTwoJoint");
        goal.name.push_back(prefix_+"_HandFingerThreeKnuckleTwoJoint");
        for (int i = 0; i < 4; i++) {
            goal.q.push_back( q(i) );
            goal.v.push_back( v(i) );
            goal.t.push_back( t(i) );
        }
        goal.maxPressure = max_pressure;
        goal.hold = hold;
        action_move_.sendGoal(goal);
    }

    bool BarrettHandInterface::waitForSuccess(double max_duration) {
        action_move_.waitForResult(ros::Duration(max_duration));
        if (action_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return true;
        }
        return false;
    }

