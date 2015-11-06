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

    int getchar2() {
        int c = getchar();
        int c2 = getchar();
        if (c2 != '\n') {
            return c2;
        }
        return c;
    }

    void spin() {

        double time_mult = 0.3;

        std::string gripper_side("right");
        std::string effector_name = gripper_side + "_HandGripLink";
        std::string hand_name = gripper_side + "_HandPalmLink";

        // initialize random seed
        srand(time(NULL));

        KDL::Frame T_W_J(KDL::Vector(0.85, 0, 1.22));

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

//        col_array.push_back( self_collision::createCollisionOctomap(oc_map, KDL::Frame()) );

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
        ros::Duration(1.0).sleep();


        bh_l.resetFingers();
        bh_r.resetFingers();

        ros::Duration(5.0).sleep();

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
        kin_model->setIgnoredJointValues(ign_joint_names, ign_q);
        showPlanerState(markers_pub_, q, col_model, kin_model);

        std::cout << "type 'c' to continue (plan)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

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

        std::cout << "type 'c' to continue (move in joint impedance)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // execute trajectory
        vi.switchToJoint();
        vi.moveJointTraj(path, 0.5, deg2rad(1 / time_mult));
        vi.waitForJoint(240 * time_mult);

        // update the robot state for planners
        vi.waitForJointState(q, ign_q);
        kin_model->setIgnoredJointValues(ign_joint_names, ign_q);
        showPlanerState(markers_pub_, q, col_model, kin_model);

        std::cout << "type 'c' to continue (plan)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

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

        std::cout << "type 'c' to continue (move in joint impedance)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // execute trajectory
        vi.moveJointTraj(path, 0.5, deg2rad(1 / time_mult));
        vi.waitForJoint(240 * time_mult);

        //
        // prepare the octomap
        //
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
        // remove the jar from the octomap
        removeNodesFromOctomap(oc_map, jar_co->geometry, T_W_J);
//        col_array.push_back( self_collision::createCollisionOctomap(oc_map, KDL::Frame()) );
        col_model->addCollisionToLink("env_link", self_collision::createCollisionOctomap(oc_map, KDL::Frame()), KDL::Frame());




        // update the robot state for planners
        vi.waitForJointState(q, ign_q);
        kin_model->setIgnoredJointValues(ign_joint_names, ign_q);
        showPlanerState(markers_pub_, q, col_model, kin_model);

        std::cout << "type 'c' to continue (plan)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

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
//        KDL::Frame T_W_Gbest;
        for (std::list<std::pair<KDL::Frame, double > >::const_iterator it = T_W_G_valid_list.begin(); it != T_W_G_valid_list.end(); it++) {
            if (max_dist < 0 || max_dist < it->second) {
                max_dist = it->second;
//                T_W_Gbest = it->first;
            }
        }

        std::list<std::pair<KDL::Frame, double > > T_W_G_valid_list2;
        for (std::list<std::pair<KDL::Frame, double > >::const_iterator it = T_W_G_valid_list.begin(); it != T_W_G_valid_list.end(); it++) {
            if (it->second > max_dist * 0.8) {
                T_W_G_valid_list2.push_back( (*it) );
            }
        }

        double closest_pose = -1;
        KDL::Frame T_W_Gbest;
        for (std::list<std::pair<KDL::Frame, double > >::const_iterator it = T_W_G_valid_list2.begin(); it != T_W_G_valid_list2.end(); it++) {
            KDL::Twist diff = KDL::diff(T_W_Gcurrent, it->first);
            double dist = diff.vel.Norm() + diff.rot.Norm() * 0.01;
            if (closest_pose < 0 || closest_pose > dist) {
                closest_pose = dist;
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

        std::cout << "type 'c' to continue (move in joint impedance)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // execute trajectory
        vi.moveJointTraj(path, 0.5, deg2rad(1 / time_mult));
        vi.waitForJoint(240 * time_mult);

        // close the fingers to the pre-grasp configuration
        if (gripper_side == "right") {
            bh_r.moveFingers(Eigen::Vector4d(0, deg2rad(55), deg2rad(55), deg2rad(55)), Eigen::Vector4d(1.2, 1.2, 1.2, 1.2), Eigen::Vector4d(3000, 3000, 3000, 3000), 1000, false);
            if (!bh_r.waitForSuccess(5.0)) {
                std::cout << "ERROR: moveFingers" << std::endl;
                return;
            }
        }
        else {
            bh_l.moveFingers(Eigen::Vector4d(0, deg2rad(55), deg2rad(55), deg2rad(55)), Eigen::Vector4d(1.2, 1.2, 1.2, 1.2), Eigen::Vector4d(3000, 3000, 3000, 3000), 1000, false);
            if (!bh_l.waitForSuccess(5.0)) {
                std::cout << "ERROR: moveFingers" << std::endl;
                return;
            }
        }

        // update the robot state for planners
        vi.waitForJointState(q, ign_q);
        kin_model->setIgnoredJointValues(ign_joint_names, ign_q);
        showPlanerState(markers_pub_, q, col_model, kin_model);

        std::cout << "type 'c' to continue (change tool and stiffness)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // move to the grasp pose (cartesian impedance)
        // first, set the tool pose for both hands
        KDL::Frame T_Wr_Gr;
        KDL::Frame T_Wl_Gl;
        {
            KDL::Frame T_W_Wr, T_W_Gr, T_W_Wl, T_W_Gl;
            kin_model->calculateFk(T_W_Wl, "left_arm_7_link", q, ign_q);
            kin_model->calculateFk(T_W_Wr, "right_arm_7_link", q, ign_q);
            kin_model->calculateFk(T_W_Gl, "left_HandGripLink", q, ign_q);
            kin_model->calculateFk(T_W_Gr, "right_HandGripLink", q, ign_q);
            T_Wl_Gl = T_W_Wl.Inverse() * T_W_Gl;
            T_Wr_Gr = T_W_Wr.Inverse() * T_W_Gr;
        }

        if (!vi.moveToolLeft(T_Wl_Gl, 0.1, 0.2)) {
            std::cout << "ERROR: moveToolLeft" << std::endl;
            return;
        }
        if (!vi.moveToolRight(T_Wr_Gr, 0.1, 0.2)) {
            std::cout << "ERROR: moveToolRight" << std::endl;
            return;
        }
        if (!vi.waitForToolMoveLeft(1.0)) {
            std::cout << "ERROR: waitForToolMoveLeft" << std::endl;
            return;
        }
        if (!vi.waitForToolMoveRight(1.0)) {
            std::cout << "ERROR: waitForToolMoveRight" << std::endl;
            return;
        }

        // set the impedance
        {
            bool result = vi.moveImpedanceLeft( 0.1, 0.2, KDL::Wrench(KDL::Vector(1000, 1000, 1000), KDL::Vector(150, 150, 150)), KDL::Wrench(KDL::Vector(0.7, 0.7, 0.7), KDL::Vector(0.7, 0.7, 0.7)) );
            result &= vi.moveImpedanceRight( 0.1, 0.2, KDL::Wrench(KDL::Vector(1000, 1000, 1000), KDL::Vector(150, 150, 150)), KDL::Wrench(KDL::Vector(0.7, 0.7, 0.7), KDL::Vector(0.7, 0.7, 0.7)) );
            result &= vi.waitForImpedanceLeft(1.0);
            result &= vi.waitForImpedanceRight(1.0);
            if (!result) {
                std::cout << "ERROR: moveImpedance" << std::endl;
                return;
            }
        }

        std::cout << "type 'c' to continue (switch to cartesian impedance)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // swith to cartesian impedance
        vi.switchToCart();

        std::cout << "type 'c' to continue (move in cartesian impedance)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // move the end effector
        {
            KDL::Frame T_W_B, T_B_Gbest;
            kin_model->calculateFk(T_W_B, "torso_base", q, ign_q);
            T_B_Gbest = T_W_B.Inverse() * T_W_Gbest;

            if (gripper_side == "right") {
                bool result = vi.moveEffectorRight(T_B_Gbest, 60.0 * time_mult, 10.0, 4.0, 0.2);
                result &= vi.waitForEffectorMoveRight(100.0 * time_mult);
                if (!result) {
                    std::cout << "ERROR: moveEffector" << std::endl;
                    return;
                }
            }
            else {
                bool result = vi.moveEffectorLeft(T_B_Gbest, 60.0 * time_mult, 10.0, 4.0, 0.2);
                result &= vi.waitForEffectorMoveLeft(100.0 * time_mult);
                if (!result) {
                    std::cout << "ERROR: moveEffector" << std::endl;
                    return;
                }
            }
        }

        std::cout << "type 'c' to continue (close the fingers)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // close the fingers to the grasp configuration
        if (gripper_side == "right") {
            bh_r.moveFingers(Eigen::Vector4d(0, deg2rad(100), deg2rad(100), deg2rad(100)), Eigen::Vector4d(1.2, 1.2, 1.2, 1.2), Eigen::Vector4d(3000, 3000, 3000, 3000), 1000, false);
            if (!bh_r.waitForSuccess(5.0)) {
                std::cout << "ERROR: moveFingers" << std::endl;
                return;
            }
        }
        else {
            bh_l.moveFingers(Eigen::Vector4d(0, deg2rad(100), deg2rad(100), deg2rad(100)), Eigen::Vector4d(1.2, 1.2, 1.2, 1.2), Eigen::Vector4d(3000, 3000, 3000, 3000), 1000, false);
            if (!bh_l.waitForSuccess(5.0)) {
                std::cout << "ERROR: moveFingers" << std::endl;
                return;
            }
        }

        // update the robot state for planners
        vi.waitForJointState(q, ign_q);
        kin_model->setIgnoredJointValues(ign_joint_names, ign_q);
        showPlanerState(markers_pub_, q, col_model, kin_model);

        std::cout << "type 'c' to continue (plan)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // move the jar out of the cabinet
        // reattach the jar to the gripper
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

        std::cout << "type 'c' to continue (move in joint impedance)" << std::endl;
        if (getchar2() != 'c' || !ros::ok()) {
            return;
        }

        // execute trajectory

        // swith to joint impedance
        vi.switchToJoint();

        vi.moveJointTraj(path, 0.5, deg2rad(1 / time_mult));
        vi.waitForJoint(240 * time_mult);

        return;
//*/
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


