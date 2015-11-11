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
#include "ompl_utilities.h"
#include "velma_robrex_utilities.h"

static const double PI = 3.141592653589793;

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
        si->setStateValidityChecker( boost::bind(&isStateValid, _1, col_model, kin_model, ndof, wcc1, wcc2) );
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

    void showTrajectory(MarkerPublisher &markers_pub,
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

            int m_id = 0;
            m_id = addRobotModelVis(markers_pub, m_id, col_model, links_fk);

            markers_pub.publish();

            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    }

    void showPlanerState(MarkerPublisher &markers_pub, const Eigen::VectorXd &q,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model) {

        const std::vector<std::string > &joint_names = kin_model->getJointNames();
        int ndof = joint_names.size();
        Eigen::VectorXd ign_q;
        std::vector<std::string > ign_joint_names;
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
        }

        int m_id = 0;
        m_id = addRobotModelVis(markers_pub, m_id, col_model, links_fk);

        markers_pub.publish();
        ros::spinOnce();
    }

    bool planTrajectorySIM(const Eigen::VectorXd &q, const Eigen::VectorXd &q_eq, const std::string &effector_name, const KDL::Frame &target_T_W_G,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        std::list<Eigen::VectorXd > &path, MarkerPublisher *markers_pub) {

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

        ros::Time last_time = ros::Time::now();
        ros::Rate loop_rate(500);
        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
        Eigen::VectorXd ign_q;
        std::vector<std::string > ign_joint_names;
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        bool goal_reached = false;
        for (int iter = 0; iter < 4000; iter++) {
            sim->oneStep(markers_pub, 3000);
//            sim->oneStep();
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

            if (markers_pub != NULL) {
                // publish markers and robot state with limited rate
                ros::Duration time_elapsed = ros::Time::now() - last_time;
                if (time_elapsed.toSec() > 0.05) {
                    // calculate forward kinematics for all links
                    for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                        kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), qq, ign_q);
                    }
                    int m_id = 0;
                    m_id = addRobotModelVis(*markers_pub, m_id, col_model, links_fk);

                    markers_pub->addEraseMarkers(m_id, m_id+300);

                    markers_pub->publish();
                    last_time = ros::Time::now();
                }
                else {
                    markers_pub->clear();
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        if (!goal_reached) {
            std::cout << "planTrajectorySIM: could not reach the goal" << std::endl;
            return false;
        }

        path = tmp_path;

        return true;
    }

