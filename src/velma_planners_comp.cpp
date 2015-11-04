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

    bool isStateValid(const ompl::base::State *s, const boost::shared_ptr<self_collision::CollisionModel > &col_model,
                        const boost::shared_ptr<KinematicModel > &kin_model, int ndof, const VelmaQ5Q6CollisionChecker &wcc1,
                        const VelmaQ5Q6CollisionChecker &wcc2) {
        Eigen::VectorXd x(ndof);
        stateOmplToEigen(s, x, ndof);

        // check collision in wrists
        if (wcc1.inCollision(x) || wcc2.inCollision(x)) {
            return false;
        }

        std::vector<self_collision::CollisionInfo> link_collisions;
        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
        }

        std::set<int> excluded_link_idx;
        return !self_collision::checkCollision(col_model, links_fk, excluded_link_idx);
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
        kin_model->setIgnoredJointValue("torso_1_joint", -90.0/180.0*PI);
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
        ros::Rate loop_rate(500);

//        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.025, 3));
        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.04, 3));

        KDL::Vector lower_bound(0.0, -0.9, 0.3);
        KDL::Vector upper_bound(1.5, 0.9, 2.2);
        if (!r_map->createDistanceMap(KDL::Vector(1.05, 0.0, 1.35), boost::bind(&checkCollision, _1, col_model, 0.04), lower_bound, upper_bound)) {
            std::cout << "could not create the distance map" << std::endl;
        }
        else {
//            std::cout << "created distance map" << std::endl;
        }

        sim->updateMetric( boost::bind(&distanceMetric, _1, _2, r_map) );

        ros::Duration(1.0).sleep();

        //
        // add tests
        //
        KDL::Frame T_W_G_lock = T_W_LOCK * KDL::Frame(KDL::Rotation::RotZ(90.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.0, -0.11, 0.0));
        KDL::Frame T_W_G_bin1 = T_W_BIN * KDL::Frame(KDL::Rotation::RotY(180.0/180.0*PI), KDL::Vector(0.0, 0.0, 0.2));
        KDL::Frame T_W_G_bin2 = T_W_BIN * KDL::Frame(KDL::Rotation::RotZ(45.0/180.0*PI) * KDL::Rotation::RotY(180.0/180.0*PI), KDL::Vector(0.0, 0.0, 0.2));
        KDL::Frame T_W_G_cab1 = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35+0.3));
        KDL::Frame T_W_G_cab2 = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35));
        KDL::Frame T_W_G_cab3 = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.15, -0.5, 1.35));
        KDL::Frame T_W_G_cab4 = KDL::Frame(KDL::Rotation::RotY(45.0/180.0*PI), KDL::Vector(1.05, 0, 2.0));
        KDL::Frame T_W_G_cab5 = KDL::Frame(KDL::Rotation::RotZ(45.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.8, 0.6, 1.8));
        KDL::Frame T_W_G_weird = KDL::Frame(KDL::Vector(0.7, -0.6, 1.8));

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
*/

        // impossible move
        ts.addNode(T_W_G_weird, q_eq, false);

        // lock -> bin
        double q_lock1_tab[] = {1.64309, 1.91875, -1.98982, 1.61324, 1.02248, 0.12445, -1.7644, -0.117448, 1.47055, 2.02187, -1.48399, -1.57693, 0.000573556, 1.5708, -1.5708};
        ts.addNode(T_W_G_bin1, q_lock1_tab, ndof, false);

        // bin -> bin
        double q_bin1_tab[] = {0.186237,    1.43153,   -1.60876,    1.64294,   0.464018,  0.0838286,   -1.16511,   0.470589,    1.56105,    1.65937,   -1.51095,   -1.58767, 0.00154277, 1.5708, -1.5708};
        ts.addNode(T_W_G_bin2, q_bin1_tab, ndof, false);

        // cab_g -> cab_d
        double q_cab_g_tab[] = {0.890049,    2.28463,   -1.47813,    2.19092,    1.46628,   0.186504,  -0.785303,  -0.501551,     1.4145,    1.68104,   -1.44994,    -1.6063, 0.00322741,     1.5708,    -1.5708};
        ts.addNode(T_W_G_cab2, q_cab_g_tab, ndof, false);

        // lock -> cab_g
        ts.addNode(T_W_G_cab2, q_lock1_tab, ndof, false);

        TestResults tr;

        std::vector<std::string > planner_names;
//        planner_names.push_back("sim");
//        planner_names.push_back("RRT*");
        planner_names.push_back("RRT-Connect");

        double rrt_max_time = 50.0;

        for (int planner_idx = 0; planner_idx < planner_names.size(); planner_idx++) {
            std::string planner_name = planner_names[planner_idx];
            std::cout << "planner_name: " << planner_name << std::endl;

            for (int try_idx = 0; try_idx < 1; try_idx++) {
                std::cout << "try_idx: " << try_idx << std::endl;
                //
                // OMPL RRT
                //
                if (planner_name == "RRT*" || planner_name == "RRT-Connect") {
                    // joint limits
                    Eigen::VectorXd lower_limit(ndof), upper_limit(ndof);
                    {
                        int q_idx = 0;
                        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {
                            lower_limit(q_idx) = kin_model->getLowerLimit(q_idx);
                            upper_limit(q_idx) = kin_model->getUpperLimit(q_idx);
//                            std::cout << lower_limit(q_idx) << "  "  << q(q_idx) << "  " << upper_limit(q_idx) << std::endl;
                        }
                    }

                    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace());
                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        space->as<ompl::base::RealVectorStateSpace>()->addDimension(joint_names[q_idx], lower_limit(q_idx), upper_limit(q_idx));
                    }

                    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
                    si->setStateValidityChecker( boost::bind(&TestDynamicModel::isStateValid, this, _1, col_model, kin_model, ndof, wrist_cc_r, wrist_cc_l) );
                    si->setStateValidityCheckingResolution(0.003);
                    si->setup();

                    ts.startTest();

                    while (ros::ok()) {
                        if (ts.isFinished()) {
                            break;
                        }

                        r_HAND_target = ts.getDestFrame();

                        // problem definition
                        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

                        // set the start state
                        if (!ts.usePrevQ()) {
                            q = ts.getInitQ();
                            dq.setZero();
                            ddq.setZero();
                        }

                        ompl::base::ScopedState<> start(space);
                        for (int q_idx = 0; q_idx < ndof; q_idx++) {
                            start[q_idx] = q(q_idx);
                        }

                        pdef->clearStartStates();
                        pdef->addStartState(start);

                        // set the goal state sampler
                        ompl::base::GoalPtr goal( new VelmaRightGripperIkGoal(si, r_HAND_target, kin_model, effector_name, boost::bind(&TestDynamicModel::isStateValid, this, _1, col_model, kin_model, ndof, wrist_cc_r, wrist_cc_l)) );
                        pdef->setGoal(goal);

                        publishTransform(br, r_HAND_target, "effector_dest", "world");

                        // planner type
                        ompl::base::PlannerPtr planner;

                        if (planner_name == "RRT-Connect") {
                            planner.reset( new ompl::geometric::RRTConnect(si) );
                        }
                        else if (planner_name == "RRT*") {
                            planner.reset( new ompl::geometric::RRTstar(si) );
                        }
                        planner->setProblemDefinition(pdef);
                        planner->setup();

                        ros::Time t_begin = ros::Time::now();
                        ompl::base::PlannerStatus status = planner->solve(rrt_max_time);
                        ros::Time t_end = ros::Time::now();

                        if (status) {// == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                            std::cout << "found solution" << std::endl;
                            ompl::base::PathPtr path = pdef->getSolutionPath();
                            std::cout << "path length: " << path->length() << std::endl;
                            boost::shared_ptr<ompl::geometric::PathGeometric > ppath = boost::static_pointer_cast<ompl::geometric::PathGeometric >(path);

                            std::list<Eigen::VectorXd > path2;
                            for (int i = 0; i< ppath->getStateCount(); i++) {
                                ompl::base::State *s = ppath->getState(i);
                                Eigen::VectorXd x(ndof);
                                stateOmplToEigen(s, x, ndof);
                                path2.push_back(x);
                            }
                            double path_len2 = 0.0;
                            bool collision = false;
                            for (double f = 0.0; f < 1.0; f += 0.01/path->length()) {
                                Eigen::VectorXd prev_q(q);
                                getPointOnPath(path2, f, q);
                                path_len2 += (q - prev_q).norm();

                                std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
                                // calculate forward kinematics for all links
                                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                                    kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                                }

                                publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                                int m_id = 0;
                                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);

                                std::set<int> excluded_link_idx;
                                if (self_collision::checkCollision(col_model, links_fk, excluded_link_idx)) {
                                    std::cout << "collision " << f << std::endl;
                                    collision = true;
                                }

                                markers_pub_.addEraseMarkers(m_id, m_id+100);

                                markers_pub_.publish();

                                ros::spinOnce();
                                ros::Duration(0.001).sleep();
                            }
                            std::cout << "path_len2 " << path_len2 << std::endl;
                            tr.addResult(planner_name, ts.getNodeId(), true, collision, (t_end - t_begin).toSec(), path->length());

                        }
                        else {
                            std::cout << "solution not found" << std::endl;
                            tr.addResult(planner_name, ts.getNodeId(), false, false, (t_end - t_begin).toSec(), -1.0);
                        }

                        // next test
                        ts.nextNode();

//                        getchar();
                    }
                }   // if (planner_name == "RRT*" || planner_name = "RRT-Connect")
                //
                // Torque-based, hierarchical task simulation
                //
                else if (planner_name == "sim") {
                    ts.startTest();

                    ros::Time t_begin, t_end;
                    Eigen::VectorXd q_prev(ndof);
                    double path_len2 = 0.0;
                    bool collision_in_prev_step = false;
                    bool collision = false;
                    bool goal_found = false;
                    bool stop = false;
                    bool first_time = true;
                    while (ros::ok() && !stop) {

                        if (loop_counter > 1500*4) {

                            if (!first_time) {
                                t_end = ros::Time::now();
                                if (goal_found) {
                                    tr.addResult(planner_name, ts.getNodeId(), true, collision, (t_end - t_begin).toSec(), path_len2);
                                }
                                else {
                                    tr.addResult(planner_name, ts.getNodeId(), false, false, (t_end - t_begin).toSec(), -1.0);
                                }
                                ts.nextNode();
                            }
                            first_time = false;
                            if (ts.isFinished()) {
                                break;
                            }

                            r_HAND_target = ts.getDestFrame();
                            if (!ts.usePrevQ()) {
                                q = ts.getInitQ();
                                dq.setZero();
                                ddq.setZero();
                                sim->setState(q, dq, ddq);
                            }

                            sim->setTarget(r_HAND_target);

                            q_prev = q;
                            path_len2 = 0.0;
                            t_begin = ros::Time::now();
                            collision = false;
                            goal_found = false;
                            if (!r_map->createDistanceMap(r_HAND_target.p, boost::bind(&checkCollision, _1, col_model, 0.04), lower_bound, upper_bound)) {
                                std::cout << "could not create the distance map" << std::endl;
                                break;
                            }
                            else {
                                std::cout << "created distance map" << std::endl;
                            }
                            loop_counter = 0;
                        }
                        loop_counter += 1;
                        publishTransform(br, r_HAND_target, "effector_dest", "world");

                        sim->oneStep();//&markers_pub_, 3000);
                        if (sim->inCollision() && !collision_in_prev_step) {
                            collision_in_prev_step = true;
                            std::cout << "collision begin" << std::endl;
                            printJointLimits(q, kin_model, joint_names);
                            collision = true;
                        }
                        else if (!sim->inCollision() && collision_in_prev_step) {
                            collision_in_prev_step = false;
                            std::cout << "collision end" << std::endl;
                        }

                        sim->getState(q, dq, ddq);
                        path_len2 += (q-q_prev).norm();
                        q_prev = q;

                        KDL::Frame current_T_B_E;
                        kin_model->calculateFk(current_T_B_E, effector_name, q);
                        KDL::Twist diff = KDL::diff(r_HAND_target, current_T_B_E);
                        if (diff.vel.Norm() < 0.015 && diff.rot.Norm() < 5.0/180.0*PI) {
                            goal_found = true;
                            std::cout << "goal reached" << std::endl;
                            std::cout << "path_len2 " << path_len2 << std::endl;
                            loop_counter = 100000;
                        }

                        // publish markers and robot state with limited rate
                        ros::Duration time_elapsed = ros::Time::now() - last_time;
                        if (time_elapsed.toSec() > 0.05) {
                            // calculate forward kinematics for all links
                            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                            }
                            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                            int m_id = 0;
                            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                            markers_pub_.publish();
                            last_time = ros::Time::now();
                            ros::spinOnce();
                        }
                        else {
                            markers_pub_.clear();
                        }
                    }
                }   // else if (planner_name == "sim")
            }   // for (try_idx)
        }   // for (planner_idx)

        for (int planner_idx = 0; planner_idx < planner_names.size(); planner_idx++) {
            std::string planner_name = planner_names[planner_idx];
            for (int nodeId = 0; nodeId < ts.getNodes(); nodeId++) {
                double successRate = tr.getSuccessRate(planner_name, nodeId);
                double meanTime = tr.getTotalMeanPlanningTime(planner_name, nodeId);
                double timeVar = tr.getTotalPlanningTimeVariance(planner_name, nodeId);
                if (successRate < 0.001) {
                    std::cout << planner_name << " nodeId: " << nodeId << " suc " << successRate << " time " << meanTime << " v " << timeVar << " path " << (-1.0) << " v " << (-1.0) << std::endl;
                }
                else {
                    double meanPath = tr.getSuccessMeanPathLength(planner_name, nodeId);
                    double pathVar = tr.getSuccessPathLengthVariance(planner_name, nodeId);
                    std::cout << planner_name << " nodeId: " << nodeId << " suc " << successRate << " time " << meanTime << " v " << timeVar << " path " << meanPath << " v " << pathVar << std::endl;
                }
            }
        }

        for (int planner_idx = 0; planner_idx < planner_names.size(); planner_idx++) {
            std::string planner_name = planner_names[planner_idx];
            for (int nodeId = 0; nodeId < ts.getNodes(); nodeId++) {
                std::vector<bool> solutionFoundVec;
                std::vector<bool> inCollisionVec;
                std::vector<double> planningTimeVec;
                std::vector<double> lengthVec;
                tr.getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
                int success_count = 0;
                for (int i = 0; i < solutionFoundVec.size(); i++) {
                    std::cout << planner_name << " nodeId: " << nodeId << " " << (solutionFoundVec[i]?"success":"failure") << " " << (inCollisionVec[i]?"inCol":"noCol") << " " << lengthVec[i] << " " << planningTimeVec[i] << std::endl;
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


