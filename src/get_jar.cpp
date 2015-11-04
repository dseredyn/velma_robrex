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

    void stateOmplToEigen(const ompl::base::State *s, Eigen::VectorXd &x, int ndof) {
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            x(q_idx) = s->as<ompl::base::RealVectorStateSpace::StateType >()->operator[](q_idx);
        }
    }

    bool isStateValidE(const Eigen::VectorXd &x, const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        int ndof, const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2) {
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

    bool isStateValid(const ompl::base::State *s, const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        int ndof, const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2) {
        Eigen::VectorXd x(ndof);
        stateOmplToEigen(s, x, ndof);
        return isStateValidE(x, col_model, kin_model, ndof, wcc1, wcc2);
    }

    bool planTrajectoryRRT(const Eigen::VectorXd &q, const std::map<std::string, double > &goal_q,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2, std::list<Eigen::VectorXd > &path2) {

        const std::vector<std::string > &joint_names = kin_model->getJointNames();
        int ndof = joint_names.size();
        path2.clear();


        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(ndof));
        ompl::base::RealVectorBounds bounds(ndof);
        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            start[q_idx] = q(q_idx);
            std::map<std::string, double >:: const_iterator it = goal_q.find( joint_names[q_idx] );
            if (it != goal_q.end()) {
                // enable joint
                goal[q_idx] = it->second;
                bounds.setLow(q_idx, kin_model->getLowerLimit(q_idx));
                bounds.setHigh(q_idx, kin_model->getUpperLimit(q_idx));
            }
            else {
                // disable joint
                goal[q_idx] = q(q_idx);
                bounds.setLow(q_idx, q(q_idx));
                bounds.setHigh(q_idx, q(q_idx));
            }
        }
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
        si->setStateValidityChecker( boost::bind(&TestDynamicModel::isStateValid, this, _1, col_model, kin_model, ndof, wcc1, wcc2) );
        si->setStateValidityCheckingResolution(1.0/180.0*PI);
        si->setup();

        if (!isStateValid(start.get(), col_model, kin_model, ndof, wcc1, wcc2)) {
            std::cout << "start state is invalid" << std::endl;
            return false;
        }
        if (!isStateValid(goal.get(), col_model, kin_model, ndof, wcc1, wcc2)) {
            std::cout << "goal state is invalid" << std::endl;
            return false;
        }

        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
        pdef->clearStartStates();
        pdef->setStartAndGoalStates(start, goal);

//        ompl::base::PlannerPtr planner(new ompl::geometric::LBTRRT(si));
        ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(si));
//        ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(si));

        planner->setProblemDefinition(pdef);
        planner->setup();

        ompl::base::PlannerStatus status = planner->solve(5.0);

        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
            std::cout << "rrt planner ok" << std::endl;


            // get rrt path
            ompl::base::PathPtr path = pdef->getSolutionPath();
            boost::shared_ptr<ompl::geometric::PathGeometric > ppath = boost::static_pointer_cast<ompl::geometric::PathGeometric >(path);
            std::cout << "path length: " << ppath->length() << std::endl;

            ompl::geometric::PathSimplifier ps(si);
            ps.simplify(*ppath.get(), 2.0);

            std::cout << "simplified path length: " << ppath->length() << std::endl;

            for (int i = 0; i< ppath->getStateCount(); i++) {
                ompl::base::State *s = ppath->getState(i);
                Eigen::VectorXd x(ndof);
                stateOmplToEigen(s, x, ndof);
                path2.push_back(x);
            }
            return true;
        }
        return false;
    }

    void showTrajectory(MarkerPublisher &markers_pub, ros::Publisher &joint_state_pub, 
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        const std::list<Eigen::VectorXd > &path) {

        if (path.size() < 2) {
            return;
        }
        const std::vector<std::string > &joint_names = kin_model->getJointNames();
        int ndof = joint_names.size();
        Eigen::VectorXd ign_q;
        std::vector<std::string > ign_joint_names;
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        double pathLength = 0.0;
        for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2 = ++path.begin(); it2 != path.end(); it1++, it2++) {
            pathLength += ( (*it2) - (*it1) ).norm();
        }

        for (double f = 0.0; f < 1.0; f += 0.005/pathLength) {
            Eigen::VectorXd x(ndof);
            getPointOnPath(path, f, x);

            std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
            }

            publishJointState(joint_state_pub, x, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub, m_id, col_model, links_fk);

            markers_pub.publish();

            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    }

    bool planTrajectorySIM(const Eigen::VectorXd &q, const Eigen::VectorXd &q_eq, const std::string &effector_name, const KDL::Frame &target_T_W_G,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        std::list<Eigen::VectorXd > &path) {

        path.clear();

        // TODO: where do these bounds should be stored?
        KDL::Vector lower_bound(-0.4, -0.9, 0.3);
        KDL::Vector upper_bound(1.5, 0.9, 2.2);

        std::list<Eigen::VectorXd > tmp_path;
        const std::vector<std::string > &joint_names = kin_model->getJointNames();
        int ndof = joint_names.size();

        Eigen::VectorXd max_q(ndof), qq(q), dq(ndof), ddq(ndof);
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

        sim->updateMetric( boost::bind(&distanceMetric, _1, _2, r_map) );

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
        }
        sim->setState(qq, dq, ddq);
        sim->setTarget(target_T_W_G);
        if (!r_map->createDistanceMap(target_T_W_G.p, boost::bind(&checkCollision, _1, col_model, 0.04), lower_bound, upper_bound)) {
            std::cout << "ERROR: planTrajectorySIM: could not create the distance map" << std::endl;
            return false;
        }
        else {
            std::cout << "planTrajectorySIM: created distance map" << std::endl;
        }

        bool goal_reached = false;
        for (int iter = 0; iter < 2000; iter++) {
            sim->oneStep();
            if (sim->inCollision()) {
                std::cout << "ERROR: planTrajectorySIM: collision" << std::endl;
                return false;
            }

            if (tmp_path.empty()) {
                tmp_path.push_back(qq);
            }
            else if ((tmp_path.back()-qq).norm() > deg2rad(5)) {
                tmp_path.push_back(qq);
            }

            sim->getState(qq, dq, ddq);
            KDL::Frame current_T_W_G;
            kin_model->calculateFk(current_T_W_G, effector_name, qq);

            KDL::Twist diff = KDL::diff(target_T_W_G, current_T_W_G);
            if (diff.vel.Norm() < 0.015 && diff.rot.Norm() < 5.0/180.0*PI) {
                std::cout << "planTrajectorySIM: goal reached" << std::endl;
                tmp_path.push_back(qq);
                goal_reached = true;
                break;
            }
        }

        if (!goal_reached) {
            return false;
        }

        tmp_path.pop_front();
        path = tmp_path;

        return true;
    }

    void spin() {
        // initialize random seed
        srand(time(NULL));

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

        // dynamics model
//        boost::shared_ptr<DynamicModel > dyn_model( new DynModelVelma() );

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
        col_array.push_back( self_collision::createCollisionOctomap(oc_map, KDL::Frame()) );

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
        Eigen::VectorXd q(ndof), dq(ndof), ddq(ndof);
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.5;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
        }

        double init_q[15] = {
        deg2rad(0),     // torso_0_joint
        deg2rad(-45),   // right_arm_0_joint
        deg2rad(-110),  // right_arm_1_joint
        deg2rad(70),    // right_arm_2_joint
        deg2rad(110),   // right_arm_3_joint
        deg2rad(0),     // right_arm_4_joint
        deg2rad(-90),   // right_arm_5_joint
        deg2rad(0),     // right_arm_6_joint
        deg2rad(45),    // left_arm_0_joint
        deg2rad(110),   // left_arm_1_joint
        deg2rad(-70),   // left_arm_2_joint
        deg2rad(-110),  // left_arm_3_joint
        deg2rad(0),     // left_arm_4_joint
        deg2rad(90),    // left_arm_5_joint
        deg2rad(0),     // left_arm_6_joint
        };

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q_eq(q_idx) = init_q[q_idx];
        }

        std::string effector_name = "right_HandGripLink";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel > kin_model( new KinematicModel(robot_description_str, joint_names) );
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

        while (true) {
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                q(q_idx) = init_q[q_idx] + randomUniform(deg2rad(-30), deg2rad(30));
                q(q_idx) = std::max(kin_model->getLowerLimit(q_idx), q(q_idx));
                q(q_idx) = std::min(kin_model->getUpperLimit(q_idx), q(q_idx));
            }
            if (isStateValidE(q, col_model, kin_model, ndof, wrist_cc_r, wrist_cc_l)) {
                break;
            }
        }

/*
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
*/
        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {
            lower_limit[q_idx] = kin_model->getLowerLimit(q_idx);
            upper_limit[q_idx] = kin_model->getUpperLimit(q_idx);
        }

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
        planTrajectoryRRT(q, goal_right, col_model, kin_model, wrist_cc_r, wrist_cc_l, path);
        showTrajectory(markers_pub_, joint_state_pub_, col_model, kin_model, path);
        q = path.back();

        planTrajectoryRRT(q, goal_left, col_model, kin_model, wrist_cc_r, wrist_cc_l, path);
        showTrajectory(markers_pub_, joint_state_pub_, col_model, kin_model, path);
        q = path.back();

        planTrajectorySIM(q, q_eq, effector_name, KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.8, 0.0, 1.22)),
                            col_model, kin_model, path);
        showTrajectory(markers_pub_, joint_state_pub_, col_model, kin_model, path);
        q = path.back();

        return;
/*
        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 50000;
        ros::Rate loop_rate(200);

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

        TestScenario ts;
        // cab_g -> cab_d
        double q_cab_g_tab[] = {0.890049,    2.28463,   -1.47813,    2.19092,    1.46628,   0.186504,  -0.785303,  -0.501551,     1.4145,    1.68104,   -1.44994,    -1.6063, 0.00322741,     1.5708,    -1.5708};
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q(q_idx) = q_cab_g_tab[q_idx];
        }
        ts.addNode(KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.8, 0.0, 1.22)), q_cab_g_tab, ndof, false);

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

        ts.startTest();

        std::string mode("target");

        sim->updateMetric( boost::bind(&distanceMetric, _1, _2, r_map) );

        KDL::Vector prev_E_pt;
        int ee_traj_m_id = 6000;

        sim->setState(q, dq, ddq);

        kin_model->calculateFk(r_HAND_target, effector_name, q_eq);

        bool collision_in_prev_step = false;
        bool stop = false;
        while (ros::ok() && !stop) {

            if (mode == "target") {
                if (loop_counter > 1500*5) {
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
        }
*/
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


