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
#include <interactive_markers/interactive_marker_server.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

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

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;
    KDL::Frame int_marker_pose_;

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

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        int_marker_pose_ = KDL::Frame(KDL::Rotation::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w), KDL::Vector(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
        printFrameKDL(int_marker_pose_);
    }

    void spin() {
        // initialize random seed
        srand(time(NULL));

        // dynamics model
        boost::shared_ptr<DynamicModel > dyn_model( new DynModelVelma() );

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

        KDL::Frame T_W_LOCK, T_W_BIN;
        createEnvironment(col_array, T_W_LOCK, T_W_BIN);

        if (!col_model->addLink("env_link", "torso_base", col_array)) {
            ROS_ERROR("ERROR: could not add external collision objects to the collision model");
            return;
        }
        col_model->generateCollisionPairs();
/*
        std::cout << "enabled collisions:" << std::endl;
        for (int i = 0; i < col_model->enabled_collisions.size(); i++) {
            std::cout << col_model->getLinkName( col_model->enabled_collisions[i].first ) << " " << col_model->getLinkName( col_model->enabled_collisions[i].second ) << std::endl;
        }

        std::cout << "disabled collisions:" << std::endl;
        for (int i = 0; i < col_model->disabled_collisions.size(); i++) {
            std::cout << col_model->getLinkName( col_model->disabled_collisions[i].first ) << " " << col_model->getLinkName( col_model->disabled_collisions[i].second ) << std::endl;
        }
        return;
*/
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

        Eigen::VectorXd q_eq(ndof);
        Eigen::VectorXd q(ndof), dq(ndof), ddq(ndof), torque(ndof);
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.5;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }

        double init_q[15] = {
        0.0,                // torso_0_joint
        90.0/180.0*PI,      // right_arm_0_joint
        -90.0/180.0*PI,     // right_arm_1_joint
        90.0/180.0*PI,     // right_arm_2_joint
        90.0/180.0*PI,     // right_arm_3_joint
        0.0/180.0*PI,       // right_arm_4_joint
        -40.0/180.0*PI,     // right_arm_5_joint
        0.0/180.0*PI,       // right_arm_6_joint
        90.0/180.0*PI,      // left_arm_0_joint
        90.0/180.0*PI,      // left_arm_1_joint
        -90.0/180.0*PI,     // left_arm_2_joint
        -90.0/180.0*PI,     // left_arm_3_joint
        0.0/180.0*PI,       // left_arm_4_joint
        90.0/180.0*PI,      // left_arm_5_joint
        -90.0/180.0*PI,     // left_arm_6_joint
        };

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q_eq(q_idx) = init_q[q_idx];
            q(q_idx) = init_q[q_idx];
        }

        std::string effector_name = "right_HandGripLink";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel > kin_model( new KinematicModel(robot_description_str, joint_names) );
//        kin_model->setIgnoredJointValue("torso_1_joint", -90.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerOneKnuckleTwoJoint", 120.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerTwoKnuckleTwoJoint", 120.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerThreeKnuckleTwoJoint", 120.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerOneKnuckleThreeJoint", 40.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerTwoKnuckleThreeJoint", 40.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerThreeKnuckleThreeJoint", 40.0/180.0*PI);
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

        Eigen::VectorXd max_q(ndof);
        max_q(0) = 10.0/180.0*PI;               // torso_0_joint
        max_q(1) = max_q(8) = 20.0/180.0*PI;    // arm_0_joint
        max_q(2) = max_q(9) = 20.0/180.0*PI;    // arm_1_joint
        max_q(3) = max_q(10) = 30.0/180.0*PI;   // arm_2_joint
        max_q(4) = max_q(11) = 40.0/180.0*PI;   // arm_3_joint
        max_q(5) = max_q(12) = 50.0/180.0*PI;   // arm_4_joint
        max_q(6) = max_q(13) = 50.0/180.0*PI;   // arm_5_joint
        max_q(7) = max_q(14) = 50.0/180.0*PI;   // arm_6_joint

        boost::shared_ptr<DynamicsSimulatorHandPose> sim(new DynamicsSimulatorHandPose(ndof, 6, effector_name, col_model, kin_model, dyn_model, joint_names, q_eq, 10000.0*max_q ) );

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 50000;
        ros::Rate loop_rate(200);

//        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.025, 3));
        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.04, 3));

        KDL::Vector lower_bound(0.0, -0.9, 0.3);
        KDL::Vector upper_bound(1.5, 0.9, 2.2);
        if (!r_map->createDistanceMap(KDL::Vector(1.05, 0.0, 1.35), boost::bind(&checkCollision, _1, col_model, 0.04), lower_bound, upper_bound)) {
            std::cout << "could not create the distance map" << std::endl;
        }
        else {
            std::cout << "created distance map" << std::endl;
        }

        //
        // add tests
        //
        KDL::Frame T_W_G_eq;
        kin_model->calculateFk(T_W_G_eq, effector_name, q_eq);
        KDL::Frame T_W_G_lock = T_W_LOCK * KDL::Frame(KDL::Rotation::RotZ(90.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.0, -0.11, 0.0));
        KDL::Frame T_W_G_bin1 = T_W_BIN * KDL::Frame(KDL::Rotation::RotY(180.0/180.0*PI), KDL::Vector(0.0, 0.0, 0.2));
        KDL::Frame T_W_G_bin2 = T_W_BIN * KDL::Frame(KDL::Rotation::RotZ(45.0/180.0*PI) * KDL::Rotation::RotY(180.0/180.0*PI), KDL::Vector(0.0, 0.0, 0.2));
        KDL::Frame T_W_G_cab1 = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35+0.3));
        KDL::Frame T_W_G_cab2 = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35));
        KDL::Frame T_W_G_cab3 = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.15, -0.5, 1.35));
        KDL::Frame T_W_G_cab4 = KDL::Frame(KDL::Rotation::RotY(45.0/180.0*PI), KDL::Vector(1.05, 0, 2.0));
        KDL::Frame T_W_G_cab5 = KDL::Frame(KDL::Rotation::RotZ(45.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.7, 0.5, 1.8));
        KDL::Frame T_W_G_weird = KDL::Frame(KDL::Vector(0.7, -0.6, 1.8));
        std::vector<KDL::Frame > vec_T_W_G;

        vec_T_W_G.push_back(T_W_G_eq);
        vec_T_W_G.push_back(T_W_G_weird);

        vec_T_W_G.push_back(T_W_G_lock);
        vec_T_W_G.push_back(T_W_G_bin1);

        vec_T_W_G.push_back(T_W_G_bin1);
        vec_T_W_G.push_back(T_W_G_bin2);

        vec_T_W_G.push_back(T_W_G_cab1);
        vec_T_W_G.push_back(T_W_G_cab2);

        vec_T_W_G.push_back(T_W_G_lock);
        vec_T_W_G.push_back(T_W_G_cab2);
/*
        vec_T_W_G.push_back(T_W_G_bin1);
        vec_T_W_G.push_back(T_W_G_bin2);
        vec_T_W_G.push_back(T_W_G_cab1);    // g
        vec_T_W_G.push_back(T_W_G_cab2);    // d
        vec_T_W_G.push_back(T_W_G_cab3);    // p
        vec_T_W_G.push_back(T_W_G_cab4);    // gg
        vec_T_W_G.push_back(T_W_G_cab5);    // l
        vec_T_W_G.push_back(T_W_G_weird);
*/

        TestScenario ts;
/*
        // single movements from the same configuration
        ts.addNode(T_W_G_weird, q_eq, false);
        ts.addNode(T_W_G_lock, q_eq, false);
        ts.addNode(T_W_G_bin1, q_eq, false);
        ts.addNode(T_W_G_bin2, q_eq, false);
        ts.addNode(T_W_G_cab1, q_eq, false);
        ts.addNode(T_W_G_cab2, q_eq, false);
        ts.addNode(T_W_G_cab3, q_eq, false);
        ts.addNode(T_W_G_cab4, q_eq, false);
        ts.addNode(T_W_G_cab5, q_eq, false);
*/
/*
        // sequence of movements
        ts.addNode(T_W_G_lock, q_eq, false);
        ts.addNode(T_W_G_bin1, q_eq, true);
        ts.addNode(T_W_G_bin2, q_eq, true);
        ts.addNode(T_W_G_lock, q_eq, true);
        ts.addNode(T_W_G_cab1, q_eq, true);
        ts.addNode(T_W_G_cab2, q_eq, true);
        ts.addNode(T_W_G_cab3, q_eq, true);
        ts.addNode(T_W_G_cab5, q_eq, true);
        ts.addNode(T_W_G_cab2, q_eq, true);
        ts.addNode(T_W_G_cab4, q_eq, true);
//*/
/*
        // impossible move
        ts.addNode(T_W_G_weird, q_eq, false);

        // lock -> bin
        double q_lock1_tab[] = {1.64309, 1.91875, -1.98982, 1.61324, 1.02248, 0.12445, -1.7644, -0.117448, 1.47055, 2.02187, -1.48399, -1.57693, 0.000573556, 1.5708, -1.5708};
        ts.addNode(T_W_G_bin1, q_lock1_tab, ndof, false);

        // bin -> bin
        double q_bin1_tab[] = {0.186237,    1.43153,   -1.60876,    1.64294,   0.464018,  0.0838286,   -1.16511,   0.470589,    1.56105,    1.65937,   -1.51095,   -1.58767, 0.00154277, 1.5708, -1.5708};
        ts.addNode(T_W_G_bin2, q_bin1_tab, ndof, false);
*/
        // cab_g -> cab_d
        double q_cab_g_tab[] = {0.890049,    2.28463,   -1.47813,    2.19092,    1.46628,   0.186504,  -0.785303,  -0.501551,     1.4145,    1.68104,   -1.44994,    -1.6063, 0.00322741,     1.5708,    -1.5708};
        ts.addNode(T_W_G_cab2, q_cab_g_tab, ndof, false);
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q(q_idx) = q_cab_g_tab[q_idx];
        }
/*
        // lock -> cab_d
        ts.addNode(T_W_G_cab2, q_lock1_tab, ndof, false);
//*/

/*        // show the hierarchy levels
        // jlc
        ts.addNode(KDL::Frame(KDL::Rotation::Quaternion(-0.0133643, 0.729555, -0.03817, 0.682725), KDL::Vector(0.442317, -0.298677, 1.56104)), q_eq, true);
        // wcc
        ts.addNode(KDL::Frame(KDL::Rotation::Quaternion(-0.289029, -0.484604, 0.730593, -0.38452), KDL::Vector(0.52431, -0.466663, 1.38939)), q_eq, true);
        // col
//        ts.addNode(KDL::Frame(KDL::Rotation::Quaternion(-0.466042, 0.488403, 0.728058, 0.119158), KDL::Vector(0.0355551, 0.248136, 1.46636)), q_eq, true);
        ts.addNode(KDL::Frame(KDL::Rotation::Quaternion(-0.466042, 0.488403, 0.728058, 0.119158), KDL::Vector(0.1, -0.1, 1.46636)), q_eq, true);
        ts.addNode(KDL::Frame(KDL::Rotation::Quaternion(-0.476391, 0.558636, 0.470781, 0.489227), KDL::Vector(0.85653, -0.109096, 1.34599)), q_eq, true);
*/
//        ts.addNode(T_W_G_cab2, q_eq, false);

        // impossible move 2
//        KDL::Frame T_W_G_weird2 = KDL::Frame(KDL::Rotation::RotX(160.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.6, -0.4, 1.8));
//        ts.addNode(T_W_G_weird2, q_eq, false);

//        ts.addNode(T_W_G_cab1, q_eq, false);

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
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }
            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
            markers_pub_.publish();
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        // TEST: distance metric
        if (false) {
//            r_map->printDistanceMap();
            showMetric(q_eq, lower_bound, upper_bound, kin_model, col_model, r_map, markers_pub_);
            return;
        }


//        ros::Duration(1.0).sleep();

        ts.startTest();

//        std::string mode("int_marker");
        std::string mode("target");

        if (mode == "int_marker") {
            sim->updateMetric( boost::bind(static_cast<KDL::Twist (*)(const KDL::Frame &, const KDL::Frame &, double)>(&KDL::diff), _1, _2, 1.0) );
        }
        else if (mode == "target") {
            sim->updateMetric( boost::bind(&distanceMetric, _1, _2, r_map) );
        }

        KDL::Vector prev_E_pt;
        int ee_traj_m_id = 6000;

        sim->setState(q, dq, ddq);

        kin_model->calculateFk(r_HAND_target, effector_name, q_eq);

        interactive_markers::InteractiveMarkerServer server("simple_marker");
        int_marker_pose_ = r_HAND_target;
        make6DofMarker(server, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, r_HAND_target, true, boost::bind(&TestDynamicModel::processFeedback, this, _1));
        server.applyChanges();

        bool collision_in_prev_step = false;
        bool stop = false;
        while (ros::ok() && !stop) {

            if (mode == "int_marker") {
                r_HAND_target = int_marker_pose_;
                sim->setTarget(r_HAND_target);
            }
            else if (mode == "target") {
                if (loop_counter > 1500*5) {
                        if (ts.isFinished()) {
                            break;
                        }

                        r_HAND_target = ts.getDestFrame();
                        if (!ts.usePrevQ()) {
                            q = ts.getInitQ();
                            dq.setZero();
                            ddq.setZero();
//                            dq(4) = 5.0;
//                            q(4) = 20.0/180.0*PI;

                            sim->setState(q, dq, ddq);
                        }

                        std::cout << "q: " << q.transpose() << std::endl;
                        ts.nextNode();

                        sim->setTarget(r_HAND_target);

                        if (!r_map->createDistanceMap(r_HAND_target.p, boost::bind(&checkCollision, _1, col_model, 0.04), lower_bound, upper_bound)) {
                            std::cout << "could not create the distance map" << std::endl;
                        }
                        else {
                            std::cout << "created distance map" << std::endl;
                        }

                        loop_counter = 0;
                }
                loop_counter += 1;
            }

            sim->oneStep(&markers_pub_, 3000);
            if (sim->inCollision() && !collision_in_prev_step) {
                collision_in_prev_step = true;
                std::cout << "collision begin" << std::endl;
                printJointLimits(q, kin_model, joint_names);
                std::cout << q.transpose() << std::endl;
                stop = true;
            }
            else if (!sim->inCollision() && collision_in_prev_step) {
                collision_in_prev_step = false;
                std::cout << "collision end" << std::endl;
            }

            sim->getState(q, dq, ddq);
            KDL::Frame current_T_B_E;
            kin_model->calculateFk(current_T_B_E, effector_name, q);

            // show the trajectory of the end effector
            if ((prev_E_pt-current_T_B_E.p).Norm() > 0.01) {
                prev_E_pt = current_T_B_E.p;
//                ee_traj_m_id = markers_pub_.addSinglePointMarker(ee_traj_m_id, current_T_B_E.p, 0, 1, 0, 1, 0.01, "world");
                markers_pub_.publish();
            }

            if (mode == "target") {
                KDL::Twist diff = KDL::diff(r_HAND_target, current_T_B_E);
                if (diff.vel.Norm() < 0.015 && diff.rot.Norm() < 5.0/180.0*PI) {
                    std::cout << "goal reached" << std::endl;
                    loop_counter = 100000;
                }
            }

            // publish markers and robot state with limited rate
            ros::Duration time_elapsed = ros::Time::now() - last_time;
            if (time_elapsed.toSec() > 0.05) {
                // show all frames
                for (int i = 0; i < vec_T_W_G.size(); i++) {
                    publishTransform(br, vec_T_W_G[i], "effector_dest_" + boost::lexical_cast<std::string>(i), "world");
                }
                publishTransform(br, r_HAND_target, "effector_dest", "world");

                // calculate forward kinematics for all links
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                }
                publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                int m_id = 0;
                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);

                KDL::Vector path_curr = current_T_B_E.p;
                bool path_stop = false;
                while (!path_stop) {
                    KDL::Vector gr, prev(path_curr);
                    if (r_map->getGradient(path_curr, gr) && (path_curr - r_HAND_target.p).Norm() > 0.05) {
                        path_curr += gr * 0.005;
                    }
                    else if ((path_curr - r_HAND_target.p).Norm() <= 0.05) {
                        path_curr = r_HAND_target.p;
                        path_stop = true;
                    }
                    else {
                        gr = r_HAND_target.p - path_curr;
                        gr.Normalize();
                        path_curr += gr * 0.005;
                    }
                    m_id = markers_pub_.addVectorMarker(m_id, prev, path_curr, 1, 1, 1, 1, 0.005, "world");
                }
                markers_pub_.addEraseMarkers(m_id, m_id+300);

//                m_id = markers_pub_.addVectorMarker(m_id, current_T_B_E.p, r_HAND_target.p, 1, 1, 1, 1, 0.005, "world");


                markers_pub_.publish();
//                markers_pub_.clear();
                last_time = ros::Time::now();
//                ros::spinOnce();
//                loop_rate.sleep();
            }
            else {
                markers_pub_.clear();
            }
            ros::spinOnce();
            loop_rate.sleep();
//            ros::Duration(1.0).sleep();
//            return;
        }

        return;
        for (int i=0; i<10; i++) {
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q_eq);
            }
            publishJointState(joint_state_pub_, q_eq, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
            markers_pub_.publish();
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


