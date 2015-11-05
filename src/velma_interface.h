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

#ifndef VELMA_INTERFACE_H__
#define VELMA_INTERFACE_H__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cartesian_trajectory_msgs/CartesianTrajectoryAction.h>

#include "Eigen/Dense"
#include <kdl/frames.hpp>

class VelmaInterface {
protected:
    enum Effector {Left, Right};

    ros::NodeHandle nh_;
    std::vector<std::string > joint_names_;
    std::vector<std::string > ign_joint_names_;
    std::map<std::string, int > j_name_idx_map_;
    std::map<std::string, int > ign_j_name_idx_map_;
    Eigen::VectorXd q_, ign_q_;
    bool q_updated_, ign_q_updated_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_jimp_;
    actionlib::SimpleActionClient<cartesian_trajectory_msgs::CartesianTrajectoryAction> action_cimp_l_;
    actionlib::SimpleActionClient<cartesian_trajectory_msgs::CartesianTrajectoryAction> action_cimp_r_;
    bool cartesian_impedance_active_, joint_impedance_active_;
    bool moveEffector(VelmaInterface::Effector ef, const KDL::Frame &T_B_E, double time, double max_force, double max_torque, double start_time);
public:
    VelmaInterface(const std::vector<std::string > &joint_names, const std::vector<std::string > &ign_joint_names);
    ~VelmaInterface();
    void jointStatesCallback(const sensor_msgs::JointState &js);
    bool waitForJointState(Eigen::VectorXd &q, Eigen::VectorXd &ign_q);
    bool moveJointTraj(const std::list<Eigen::VectorXd > &traj, double start_time, double speed);
    bool waitForJoint(double max_duration);
    bool switchToJoint();
    bool switchToCart();
};

#endif  // VELMA_INTERFACE_H__

