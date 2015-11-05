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

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "barrett_hand_controller_msgs/Empty.h"
#include <barrett_hand_controller_msgs/BHMoveAction.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Path.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/PathSimplifier.h>

#include "velma_dyn_model.h"
#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/task_col.h"
#include "planer_utils/task_hand.h"
#include "planer_utils/task_jlc.h"
#include "planer_utils/task_wcc.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"
#include "planer_utils/simulator.h"
#include "planer_utils/reachability_map.h"

#include "experiments_utilities.h"
#include "ompl_utilities.h"
#include "velma_robrex_utilities.h"
#include "barrett_hand_interface.h"
#include "velma_interface.h"

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

public:
    TestDynamicModel() :
        nh_(),
        PI(3.141592653589793),
        markers_pub_(nh_)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }

    ~TestDynamicModel() {
    }

    void spin() {

        std::string gripper_side("right");
        std::string effector_name = gripper_side + "_HandGripLink";
        std::string hand_name = gripper_side + "_HandPalmLink";

        // initialize random seed
        srand(time(NULL));

        KDL::Frame T_W_J(KDL::Vector(0.85, -0.2, 1.22));

        // prepare the octomap
        boost::shared_ptr<octomap::OcTree > oc_map;
        {
            // get the octomap from the server
            octomap_msgs::GetOctomap get_oc_map;
            if (!ros::service::call("/octomap_full", get_oc_map)) {
                std::cout << "ERROR: ros::service::call(\"/octomap_full\" " << std::endl;
                return;
            }
            oc_map.reset( static_cast<octomap::OcTree* >( octomap_msgs::fullMsgToMap( get_oc_map.response.map ) ) );

            // delete all unnecessary nodes
            oc_map->expand();
            int occupied_count = 0;
            std::list<octomap::OcTreeKey > del_key_list;
            for (octomap::OcTree::leaf_iterator it = oc_map->begin_leafs(); it != oc_map->end_leafs(); it++) {
                if(it->getOccupancy() <= oc_map->getOccupancyThres())
                {
                    del_key_list.push_back(it.getKey());
                    continue;
                }
                occupied_count++;
            }
            for (std::list<octomap::OcTreeKey >::const_iterator it = del_key_list.begin(); it != del_key_list.end(); it++) {
                oc_map->deleteNode( (*it) );
            }
        }

        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

        //
        // collision model
        //
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);
	    col_model->parseSRDF(robot_semantic_description_str);
        col_model->generateCollisionPairs();

        // external collision objects - part of virtual link connected to the base link
        self_collision::Link::VecPtrCollision col_array;

//        KDL::Frame T_W_LOCK, T_W_BIN;
//        createEnvironment(col_array, T_W_LOCK, T_W_BIN);

        boost::shared_ptr< self_collision::Collision > jar_co = self_collision::createCollisionCapsule(0.045, 0.07, T_W_J);
        col_array.push_back( jar_co );

        col_array.push_back( self_collision::createCollisionOctomap(oc_map, KDL::Frame()) );

        if (!col_model->addLink("env_link", "torso_base", col_array)) {
            ROS_ERROR("ERROR: could not add external collision objects to the collision model");
            return;
        }
        col_model->generateCollisionPairs();

        // set colors for each link
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            for (self_collision::Link::VecPtrCollision::iterator it = col_model->getLink(l_idx)->collision_array.begin(); it != col_model->getLink(l_idx)->collision_array.end(); it++) {
                (*it)->geometry->setColor(0,1,0,0.5);
            }
        }

        // remove the jar from the octomap
        removeNodesFromOctomap(oc_map, jar_co->geometry, T_W_J);

        VelmaQ5Q6CollisionChecker wrist_cc_r(6, 7, 0.2, false);
        VelmaQ5Q6CollisionChecker wrist_cc_l(13, 14, 0.2, true);

        //
        // robot state
        //
        std::vector<std::string > joint_names;
        joint_names.push_back("torso_0_joint");
        joint_names.push_back("right_arm_0_joint");
        joint_names.push_back("right_arm_1_joint");
        joint_names.push_back("right_arm_2_joint");
        joint_names.push_back("right_arm_3_joint");
        joint_names.push_back("right_arm_4_joint");
        joint_names.push_back("right_arm_5_joint");
        joint_names.push_back("right_arm_6_joint");
        joint_names.push_back("left_arm_0_joint");
        joint_names.push_back("left_arm_1_joint");
        joint_names.push_back("left_arm_2_joint");
        joint_names.push_back("left_arm_3_joint");
        joint_names.push_back("left_arm_4_joint");
        joint_names.push_back("left_arm_5_joint");
        joint_names.push_back("left_arm_6_joint");

        int ndof = joint_names.size();

        // equilibrium pose
        Eigen::VectorXd q_eq(ndof);
        Eigen::VectorXd q(ndof);

        double q_eq_vec[15] = {
            deg2rad(0),
            deg2rad(-45), deg2rad(-110), deg2rad(70), deg2rad(110), deg2rad(0), deg2rad(-90), deg2rad(0),
            deg2rad(45), deg2rad(110), deg2rad(-70), deg2rad(-110), deg2rad(0), deg2rad(90), deg2rad(0),
        };

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q_eq(q_idx) = q_eq_vec[q_idx];
        }

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel > kin_model( new KinematicModel(robot_description_str, joint_names) );
        Eigen::VectorXd ign_q;
        std::vector<std::string > ign_joint_names;
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        kin_model->setLowerLimit(0, -110.0/180.0*PI);
        kin_model->setUpperLimit(0, 110.0/180.0*PI);
        kin_model->setUpperLimit(2, -5.0/180.0*PI);
        kin_model->setLowerLimit(4, 5.0/180.0*PI);
        kin_model->setUpperLimit(6, -5.0/180.0*PI);
        kin_model->setLowerLimit(9, 5.0/180.0*PI);
        kin_model->setUpperLimit(11, -5.0/180.0*PI);
        kin_model->setLowerLimit(13, 5.0/180.0*PI);

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        VelmaInterface vi(joint_names, ign_joint_names);

        BarrettHandInterface bh_l("left");
        BarrettHandInterface bh_r("right");

        bh_l.resetFingers();
        bh_r.resetFingers();

        ros::Duration(2.0).sleep();

        bh_l.moveFingers(Eigen::Vector4d(0, deg2rad(130), deg2rad(130), deg2rad(130)), Eigen::Vector4d(1.2, 1.2, 1.2, 1.2), Eigen::Vector4d(3000, 3000, 3000, 3000), 1000, false);
        bh_r.moveFingers(Eigen::Vector4d(0, deg2rad(130), deg2rad(130), deg2rad(130)), Eigen::Vector4d(1.2, 1.2, 1.2, 1.2), Eigen::Vector4d(3000, 3000, 3000, 3000), 1000, false);

        if (!bh_l.waitForSuccess(5.0)) {
            std::cout << "ERROR: bh_l.moveFingers" << std::endl;
            return;
        }
        if (!bh_r.waitForSuccess(5.0)) {
            std::cout << "ERROR: bh_r.moveFingers" << std::endl;
            return;
        }

        vi.waitForJointState(q, ign_q);

        for (int idx = 0; idx < ign_joint_names.size(); idx++) {
            kin_model->setIgnoredJointValue(ign_joint_names[idx], ign_q(idx));
        }

        showPlanerState(markers_pub_, q, col_model, kin_model);

        // move the robot to initial configuration
        std::map<std::string, double > goal_right;
        std::map<std::string, double > goal_left;
        goal_right.insert( std::make_pair(std::string("right_arm_0_joint"), deg2rad(-45)) );
        goal_right.insert( std::make_pair(std::string("right_arm_1_joint"), deg2rad(-110)) );
        goal_right.insert( std::make_pair(std::string("right_arm_2_joint"), deg2rad(70)) );
        goal_right.insert( std::make_pair(std::string("right_arm_3_joint"), deg2rad(110)) );
        goal_right.insert( std::make_pair(std::string("right_arm_4_joint"), deg2rad(0)) );
        goal_right.insert( std::make_pair(std::string("right_arm_5_joint"), deg2rad(-90)) );
        goal_right.insert( std::make_pair(std::string("right_arm_6_joint"), deg2rad(0)) );

        goal_left.insert( std::make_pair(std::string("left_arm_0_joint"), deg2rad(45)) );
        goal_left.insert( std::make_pair(std::string("left_arm_1_joint"), deg2rad(110)) );
        goal_left.insert( std::make_pair(std::string("left_arm_2_joint"), deg2rad(-70)) );
        goal_left.insert( std::make_pair(std::string("left_arm_3_joint"), deg2rad(-110)) );
        goal_left.insert( std::make_pair(std::string("left_arm_4_joint"), deg2rad(0)) );
        goal_left.insert( std::make_pair(std::string("left_arm_5_joint"), deg2rad(90)) );
        goal_left.insert( std::make_pair(std::string("left_arm_6_joint"), deg2rad(0)) );

        std::list<Eigen::VectorXd > path;
        while (ros::ok()) {
            if (planTrajectoryRRT(q, goal_right, col_model, kin_model, wrist_cc_r, wrist_cc_l, path)) {
                break;
            }
            std::cout << "RRT failure, trying again..." << std::endl;
        }
        if (!ros::ok()) {
            return;
        }
        showTrajectory(markers_pub_, col_model, kin_model, path);
        q = path.back();

        // execute trajectory
        vi.switchToJoint();
        vi.moveJointTraj(path, 0.5, deg2rad(5));
        vi.waitForJoint(60);

        while (ros::ok()) {
            if (planTrajectoryRRT(q, goal_left, col_model, kin_model, wrist_cc_r, wrist_cc_l, path)) {
                break;
            }
            std::cout << "RRT failure, trying again..." << std::endl;
        }
        if (!ros::ok()) {
            return;
        }
        showTrajectory(markers_pub_, col_model, kin_model, path);
        q = path.back();

        // execute trajectory
        vi.moveJointTraj(path, 0.5, deg2rad(5));
        vi.waitForJoint(60);

        // generate grasps for the jar
        std::list<KDL::Frame> T_W_G_list;
        for (double grip_angle = 0.0; grip_angle < deg2rad(359); grip_angle += deg2rad(5.0)) {
            T_W_G_list.push_back(
                T_W_J * KDL::Frame(KDL::Rotation::RotZ(grip_angle)) * KDL::Frame(KDL::Rotation::RotY(deg2rad(90))) * KDL::Frame(KDL::Vector(0,0,-0.03)) );
            T_W_G_list.push_back(
                T_W_J * KDL::Frame(KDL::Rotation::RotZ(grip_angle)) * KDL::Frame(KDL::Rotation::RotY(deg2rad(90))) * KDL::Frame(KDL::Vector(0,0,-0.03)) * KDL::Frame(KDL::Rotation::RotZ(deg2rad(180))) );
        }

        double grip_backoff = 0.14;
        std::list<std::pair<KDL::Frame, double > > T_W_G_valid_list;
        std::map<std::string, double > hand_q_map;
        hand_q_map[gripper_side+"_HandFingerOneKnuckleOneJoint"] = deg2rad(0.0);
        hand_q_map[gripper_side+"_HandFingerOneKnuckleTwoJoint"] = deg2rad(55.0);
        hand_q_map[gripper_side+"_HandFingerOneKnuckleThreeJoint"] = hand_q_map["left_HandFingerOneKnuckleTwoJoint"] * 0.333333;
        hand_q_map[gripper_side+"_HandFingerTwoKnuckleTwoJoint"] = deg2rad(55.0);
        hand_q_map[gripper_side+"_HandFingerTwoKnuckleThreeJoint"] = hand_q_map["left_HandFingerTwoKnuckleTwoJoint"] * 0.333333;
        hand_q_map[gripper_side+"_HandFingerThreeKnuckleTwoJoint"] = deg2rad(55.0);
        hand_q_map[gripper_side+"_HandFingerThreeKnuckleThreeJoint"] = hand_q_map["left_HandFingerThreeKnuckleTwoJoint"] * 0.333333;

        KDL::Frame T_W_Gcurrent;
        kin_model->calculateFk(T_W_Gcurrent, effector_name, q, ign_q);

        for (std::list<KDL::Frame>::const_iterator it = T_W_G_list.begin(); it != T_W_G_list.end(); it++) {
            const KDL::Frame &T_W_G = (*it);
            KDL::Twist diff = KDL::diff(T_W_Gcurrent, (*it));
            if (diff.rot.Norm() > deg2rad(90)) {
                continue;
            }

            KDL::Frame T_W_E = T_W_G * KDL::Frame(KDL::Vector(0,0,-0.12));
            bool collision = false;
            double min_dist = -1.0;
            for (double backoff = 0.0; backoff <= grip_backoff; backoff += 0.2) {
                double dist;
                if (checkSubtreeCollision( col_model, kin_model, hand_q_map, hand_name, T_W_E * KDL::Frame(KDL::Vector(0,0,-backoff)), col_model->getLink("env_link"), KDL::Frame(), dist)) {
                    collision = true;
                    break;
                }
                if (min_dist < 0 || dist < min_dist) {
                    min_dist = dist;
                }
            }
            if (!collision) {
                T_W_G_valid_list.push_back(std::make_pair(T_W_G, min_dist));
            }
        }
        if (T_W_G_valid_list.empty()) {
            std::cout << "could not find valid grasps" << std::endl;
            return;
        }

        double max_dist = -1;
        KDL::Frame T_W_Gbest;
        for (std::list<std::pair<KDL::Frame, double > >::const_iterator it = T_W_G_valid_list.begin(); it != T_W_G_valid_list.end(); it++) {
            if (max_dist < 0 || max_dist < it->second) {
                max_dist = it->second;
                T_W_Gbest = it->first;
            }
        }
        std::cout << "min_dist: " << max_dist << std::endl;
        publishTransform(br, T_W_Gbest, std::string("effector_dest"), "world");

        // move to the pre-grasp pose
        KDL::Frame r_HAND_target = T_W_Gbest * KDL::Frame(KDL::Vector(0,0,-grip_backoff));
        if (!planTrajectorySIM(q, q_eq, effector_name, r_HAND_target, col_model, kin_model, path)) {
            std::cout << "SIM failure" << std::endl;
            return;
        }
        showTrajectory(markers_pub_, col_model, kin_model, path);
        q = path.back();

        // close the fingers to the pre-grasp configuration
        for (std::map<std::string, double >::const_iterator it = hand_q_map.begin(); it != hand_q_map.end(); it++) {
            kin_model->setIgnoredJointValue(it->first, it->second);
        }
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        // move to the grasp pose (cartesian impedance)
        // TODO

        // close the fingers to the grasp configuration
        // TODO

        // move the jar out of the cabinet
        col_model->removeCollisionFromLink("env_link", jar_co);

        KDL::Frame T_W_Ecurrent;
        kin_model->calculateFk(T_W_Ecurrent, hand_name, q, ign_q);
        KDL::Frame T_E_J = T_W_Ecurrent.Inverse() * T_W_J;
        jar_co->origin = T_E_J;
        col_model->addCollisionToLink(hand_name, jar_co, T_E_J);

        r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(deg2rad(60)) * KDL::Rotation::RotY(deg2rad(90)), KDL::Vector(0.45, -0.2, 1.3));
        if (!planTrajectorySIM(q, q_eq, effector_name, r_HAND_target, col_model, kin_model, path)) {
            std::cout << "SIM failure" << std::endl;
            return;
        }
        showTrajectory(markers_pub_, col_model, kin_model, path);
        q = path.back();

        return;

/*
        for (int i = 0; i < 100; i++) {
            // publish markers and robot state with limited rate
            publishTransform(br, r_HAND_target, "effector_dest", "world");

            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q, ign_q);
            }
            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);

            markers_pub_.addEraseMarkers(m_id, m_id+300);

            markers_pub_.publish();

            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }

        return;
*/

        Eigen::VectorXd dq(ndof), ddq(ndof);
        dq.setZero();
        ddq.setZero();

        Eigen::VectorXd max_q(ndof);
        max_q(0) = 10.0/180.0*PI;               // torso_0_joint
        max_q(1) = max_q(8) = 20.0/180.0*PI;    // arm_0_joint
        max_q(2) = max_q(9) = 20.0/180.0*PI;    // arm_1_joint
        max_q(3) = max_q(10) = 30.0/180.0*PI;   // arm_2_joint
        max_q(4) = max_q(11) = 40.0/180.0*PI;   // arm_3_joint
        max_q(5) = max_q(12) = 50.0/180.0*PI;   // arm_4_joint
        max_q(6) = max_q(13) = 50.0/180.0*PI;   // arm_5_joint
        max_q(7) = max_q(14) = 50.0/180.0*PI;   // arm_6_joint

        // create dynamics model
        boost::shared_ptr<DynamicModel > dyn_model( new DynModelVelma() );
        boost::shared_ptr<DynamicsSimulatorHandPose> sim(new DynamicsSimulatorHandPose(ndof, 6, effector_name, col_model, kin_model, dyn_model, joint_names, q_eq, 10000.0*max_q ) );
        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.04, 3));

        // loop variables
        ros::Time last_time = ros::Time::now();
        ros::Rate loop_rate(200);

        // set colors for each link
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            for (self_collision::Link::VecPtrCollision::iterator it = col_model->getLink(l_idx)->collision_array.begin(); it != col_model->getLink(l_idx)->collision_array.end(); it++) {
                (*it)->geometry->setColor(0,1,0,0.5);
            }
        }

        // show the initial configuration for 2 seconds.
        for (int i=0; i<20; i++) {
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q, ign_q);
            }
//            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
            markers_pub_.publish();
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        sim->updateMetric( boost::bind(&distanceMetric, _1, _2, r_map) );

        sim->setState(q, dq, ddq);
        sim->setTarget(r_HAND_target);

        KDL::Vector lower_bound(-0.4, -0.9, 0.3);
        KDL::Vector upper_bound(1.5, 0.9, 2.2);
        if (!r_map->createDistanceMap(r_HAND_target.p, boost::bind(&checkCollision, _1, col_model, 0.04), lower_bound, upper_bound)) {
            std::cout << "could not create the distance map" << std::endl;
        }
        else {
            std::cout << "created distance map" << std::endl;
        }

        bool stop = false;
        while (ros::ok() && !stop) {

            sim->oneStep(&markers_pub_, 3000);
            if (sim->inCollision()) {
                std::cout << "collision" << std::endl;
                printJointLimits(q, kin_model, joint_names);
                std::cout << q.transpose() << std::endl;
                stop = true;
            }

            sim->getState(q, dq, ddq);
            KDL::Frame current_T_B_E;
            kin_model->calculateFk(current_T_B_E, effector_name, q, ign_q);

            KDL::Twist diff = KDL::diff(r_HAND_target, current_T_B_E);
            if (diff.vel.Norm() < 0.015 && diff.rot.Norm() < 5.0/180.0*PI) {
                std::cout << "goal reached" << std::endl;
                stop = true;
            }

            // publish markers and robot state with limited rate
            ros::Duration time_elapsed = ros::Time::now() - last_time;
            if (time_elapsed.toSec() > 0.05) {
                publishTransform(br, r_HAND_target, "effector_dest", "world");

                // calculate forward kinematics for all links
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q, ign_q);
                }
//                publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                int m_id = 0;
                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);

                markers_pub_.addEraseMarkers(m_id, m_id+300);

                markers_pub_.publish();
                last_time = ros::Time::now();
            }
            else {
                markers_pub_.clear();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }



//*/
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


