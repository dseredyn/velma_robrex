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
#ifndef VELMA_ROBREX_UTILITIES_H__
#define VELMA_ROBREX_UTILITIES_H__

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

    bool isStateValidE(const Eigen::VectorXd &x, const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        int ndof, const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2);

    bool isStateValid(const ompl::base::State *s, const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        int ndof, const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2);

    bool planTrajectoryRRT(const Eigen::VectorXd &q, const std::map<std::string, double > &goal_q,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2, std::list<Eigen::VectorXd > &path2);

    void showTrajectory(MarkerPublisher &markers_pub, 
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        const std::list<Eigen::VectorXd > &path);

    void showPlanerState(MarkerPublisher &markers_pub, const Eigen::VectorXd &q,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model);

    bool planTrajectorySIM(const Eigen::VectorXd &q, const Eigen::VectorXd &q_eq, const std::string &effector_name, const KDL::Frame &target_T_W_G,
                        const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        std::list<Eigen::VectorXd > &path, MarkerPublisher *markers_pub=NULL);

#endif  // VELMA_ROBREX_UTILITIES_H__

