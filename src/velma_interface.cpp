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

#include "velma_interface.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <controller_manager_msgs/SwitchController.h>
#include "planer_utils/utilities.h"

    VelmaInterface::VelmaInterface(const std::vector<std::string > &joint_names, const std::vector<std::string > &ign_joint_names) :
        nh_(),
        joint_names_(joint_names),
        ign_joint_names_(ign_joint_names),
        q_(joint_names.size()),
        ign_q_(ign_joint_names.size()),
        action_jimp_("/spline_trajectory_action_joint", true),
        action_cimp_l_("/left_arm/cartesian_trajectory", true),
        action_cimp_r_("/right_arm/cartesian_trajectory", true),
        cartesian_impedance_active_(false),
        joint_impedance_active_(false)
    {
        for (int idx = 0; idx < joint_names.size(); idx++) {
            j_name_idx_map_[joint_names[idx]] = idx;
        }
        for (int idx = 0; idx < ign_joint_names.size(); idx++) {
            ign_j_name_idx_map_[ign_joint_names[idx]] = idx;
        }
    }

    VelmaInterface::~VelmaInterface() {
    }

    void VelmaInterface::jointStatesCallback(const sensor_msgs::JointState &js) {
        for (int idx = 0; idx < js.name.size(); idx++) {
            std::map<std::string, int >::const_iterator it = j_name_idx_map_.find( js.name[idx] );
            if (it != j_name_idx_map_.end()) {
                q_( it->second ) = js.position[idx];
                q_updated_ = true;
                continue;
            }
            it = ign_j_name_idx_map_.find( js.name[idx] );
            if (it != ign_j_name_idx_map_.end()) {
                ign_q_( it->second ) = js.position[idx];
                ign_q_updated_ = true;
            }
        }
    }

    bool VelmaInterface::waitForJointState(Eigen::VectorXd &q, Eigen::VectorXd &ign_q) {
        q_updated_ = false;
        ign_q_updated_ = false;
        ros::Subscriber sub = nh_.subscribe("/joint_states", 1000, &VelmaInterface::jointStatesCallback, this);
        while ( !(q_updated_ && ign_q_updated_) && ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        sub.shutdown();
        q = q_;
        ign_q = ign_q_;
    }

    bool VelmaInterface::moveJointTraj(const std::list<Eigen::VectorXd > &traj, double start_time, double speed) {

        int steps = traj.size();
        if (steps < 2) {
            return false;
        }

        double pathLength = 0.0;
        for (std::list<Eigen::VectorXd >::const_iterator it1 = traj.begin(), it2 = ++traj.begin(); it2 != traj.end(); it1++, it2++) {
            pathLength += ( (*it2) - (*it1) ).norm();
        }

        double total_time = pathLength / speed;

        if ( !(!cartesian_impedance_active_ && joint_impedance_active_) ) {
            std::cout << "ERROR: moveJointTraj: wrong mode" << std::endl;
            return false;
        }
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = joint_names_;

        double time = 0.0;
        for (std::list<Eigen::VectorXd >::const_iterator it = traj.begin(); it != traj.end(); it++) {
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions.resize(joint_names_.size());
            for (int q_idx = 0; q_idx < joint_names_.size(); q_idx++) {
                pt.positions[q_idx] = (*it)(q_idx);
            }
            pt.time_from_start = ros::Duration(time);
            goal.trajectory.points.push_back(pt);
            time += total_time / static_cast<double>(steps);
        }

        for (int q_idx = 0; q_idx < joint_names_.size(); q_idx++) {
            control_msgs::JointTolerance path_tol;
            path_tol.name = joint_names_[q_idx];
            path_tol.position = deg2rad(5.0);
            path_tol.velocity = deg2rad(5.0);
            path_tol.acceleration = deg2rad(5.0);
            goal.path_tolerance.push_back(path_tol);
            goal.goal_tolerance.push_back(path_tol);
        }

        goal.goal_time_tolerance = ros::Duration(0.5);
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(start_time);
        action_jimp_.sendGoal(goal);
        return true;
    }

    bool VelmaInterface::waitForJoint(double max_duration) {
        action_jimp_.waitForResult(ros::Duration(max_duration));
        if (action_jimp_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return true;
        }
        return false;
    }

    bool VelmaInterface::switchToJoint() {
        cartesian_impedance_active_ = false;

        controller_manager_msgs::SwitchController switch_cmd;
        switch_cmd.request.start_controllers.push_back("JntImp");
        switch_cmd.request.start_controllers.push_back("TrajectoryGeneratorJoint");
        switch_cmd.request.stop_controllers.push_back("CImp");
        switch_cmd.request.stop_controllers.push_back("PoseIntLeft");
        switch_cmd.request.stop_controllers.push_back("PoseIntRight");
        switch_cmd.request.strictness = 2;//STRICT;
        if (!ros::service::call("/controller_manager/switch_controller", switch_cmd)) {
            std::cout << "ERROR: ros::service::call(\"/controller_manager/switch_controller\" " << std::endl;
        }

        if (switch_cmd.response.ok) {        
            joint_impedance_active_ = true;
            return true;
        }
        std::cout << "ERROR: switchToJoint failed" << std::endl;
        return false;
    }

    bool VelmaInterface::switchToCart() {
        joint_impedance_active_ = false;

        controller_manager_msgs::SwitchController switch_cmd;
        switch_cmd.request.start_controllers.push_back("CImp");
        switch_cmd.request.start_controllers.push_back("PoseIntLeft");
        switch_cmd.request.start_controllers.push_back("PoseIntRight");
        switch_cmd.request.stop_controllers.push_back("JntImp");
        switch_cmd.request.stop_controllers.push_back("TrajectoryGeneratorJoint");
        switch_cmd.request.strictness = 2;//STRICT;
        if (!ros::service::call("/controller_manager/switch_controller", switch_cmd)) {
            std::cout << "ERROR: ros::service::call(\"/controller_manager/switch_controller\" " << std::endl;
        }

        if (switch_cmd.response.ok) {        
            cartesian_impedance_active_ = true;
            return true;
        }
        std::cout << "ERROR: switchToCart failed" << std::endl;
        return false;
    }
/*
    bool VelmaInterface::isJointImpedanceActive(self):
        if self.joint_impedance_active and not self.cartesian_impedance_active:
            return True
        return False

    bool VelmaInterface::isCartesianImpedanceActive(self):
        if not self.joint_impedance_active and self.cartesian_impedance_active:
            return True
        return False
*/

    bool VelmaInterface::moveEffector(VelmaInterface::Effector ef, const KDL::Frame &T_B_E, double time, double max_force, double max_torque, double start_time) {
        if ( !(cartesian_impedance_active_ && !joint_impedance_active_) ) {
            std::cout << "ERROR: moveWrist: wrong mode" << std::endl;
            return false;
        }
        cartesian_trajectory_msgs::CartesianTrajectoryGoal goal;

        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(start_time);
        cartesian_trajectory_msgs::CartesianTrajectoryPoint pt;
        pt.time_from_start = ros::Duration(time);
        pt.pose.position.x = T_B_E.p.x();
        pt.pose.position.y = T_B_E.p.y();
        pt.pose.position.z = T_B_E.p.z();
        double qx, qy, qz, qw;
        T_B_E.M.GetQuaternion(qx, qy, qx, qw);
        pt.pose.orientation.x = qx;
        pt.pose.orientation.y = qy;
        pt.pose.orientation.z = qz;
        pt.pose.orientation.w = qw;
        pt.twist.linear.x = 0.0;
        pt.twist.linear.y = 0.0;
        pt.twist.linear.z = 0.0;
        pt.twist.angular.x = 0.0;
        pt.twist.angular.y = 0.0;
        pt.twist.angular.z = 0.0;
        goal.trajectory.points.push_back(pt);

        goal.wrench_constraint.force.x = max_force;
        goal.wrench_constraint.force.y = max_force;
        goal.wrench_constraint.force.z = max_force;
        goal.wrench_constraint.torque.x = max_torque;
        goal.wrench_constraint.torque.y = max_torque;
        goal.wrench_constraint.torque.z = max_torque;
        goal.path_tolerance.position.x = 0.01;
        goal.path_tolerance.position.y = 0.01;
        goal.path_tolerance.position.z = 0.01;
        goal.path_tolerance.rotation.x = 0.01;
        goal.path_tolerance.rotation.y = 0.01;
        goal.path_tolerance.rotation.z = 0.01;
        goal.goal_tolerance = goal.path_tolerance;

        if (ef == Left) {
            action_cimp_l_.sendGoal(goal);
        }
        else if (ef == Right) {
            action_cimp_r_.sendGoal(goal);
        }
        else {
            return false;
        }
        return true;
    }

