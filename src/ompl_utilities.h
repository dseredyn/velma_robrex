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

//#include <ros/ros.h>
//#include "ros/package.h"
//#include <sensor_msgs/JointState.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <tf/transform_broadcaster.h>
//#include <interactive_markers/interactive_marker_server.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
//#include "Eigen/LU"

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

//#include "velma_dyn_model.h"
//#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
//#include "planer_utils/marker_publisher.h"
//#include "planer_utils/random_uniform.h"
//#include "planer_utils/utilities.h"
//#include "planer_utils/simulator.h"
//#include "planer_utils/reachability_map.h"

//#include "experiments_utilities.h"

void stateOmplToEigen(const ompl::base::State *s, Eigen::VectorXd &x, int ndof);
void stateEigenToOmpl(const Eigen::VectorXd &x, ompl::base::State *s, int ndof);

class VelmaRightGripperIkGoal : public ompl::base::GoalSampleableRegion {
public:

    VelmaRightGripperIkGoal(const ompl::base::SpaceInformationPtr &si, const KDL::Frame &T_W_G_dest,
                            const boost::shared_ptr<KinematicModel> &kin_model, const std::string &effector_name,
                            const ompl::base::StateValidityCheckerFn &svc);

    virtual void sampleGoal (ompl::base::State *st) const;

    virtual unsigned int maxSampleCount () const;

    virtual bool couldSample () const;

    virtual double distanceGoal (const ompl::base::State *st) const;

protected:
    KDL::Frame T_W_G_dest_;
    const boost::shared_ptr<KinematicModel> &kin_model_;
    int ndof_;
    std::string effector_name_;
    ompl::base::StateValidityCheckerFn svc_;
};

