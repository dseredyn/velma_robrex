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

#include "ompl_utilities.h"
#include "experiments_utilities.h"

void stateOmplToEigen(const ompl::base::State *s, Eigen::VectorXd &x, int ndof) {
    for (int q_idx = 0; q_idx < ndof; q_idx++) {
        x(q_idx) = s->as<ompl::base::RealVectorStateSpace::StateType >()->operator[](q_idx);
    }
}

void stateEigenToOmpl(const Eigen::VectorXd &x, ompl::base::State *s, int ndof) {
    for (int q_idx = 0; q_idx < ndof; q_idx++) {
        s->as<ompl::base::RealVectorStateSpace::StateType >()->operator[](q_idx) = x(q_idx);
    }
}

VelmaRightGripperIkGoal::VelmaRightGripperIkGoal(const ompl::base::SpaceInformationPtr &si, const KDL::Frame &T_W_G_dest,
                                                    const boost::shared_ptr<KinematicModel> &kin_model, const std::string &effector_name,
                                                    const ompl::base::StateValidityCheckerFn &svc) :
        GoalSampleableRegion(si),
        T_W_G_dest_(T_W_G_dest),
        kin_model_(kin_model),
        ndof_(kin_model_->getDofCount()),
        effector_name_(effector_name),
        svc_(svc)
{
}

void VelmaRightGripperIkGoal::sampleGoal (ompl::base::State *st) const {
        Eigen::VectorXd ik_q( ndof_ );
        while (true) {
            if (randomizedIkSolution(kin_model_, T_W_G_dest_, ik_q)) {
                stateEigenToOmpl(ik_q, st, ndof_);
                if (svc_(st)){
                    std::cout << "VelmaRightGripperIkGoal::sampleGoal: found ik solution" << std::endl;
                    break;
                }
            }
        }
}

unsigned int VelmaRightGripperIkGoal::maxSampleCount () const {
        return 1000000;
}

bool VelmaRightGripperIkGoal::couldSample () const {
        return true;
}

double VelmaRightGripperIkGoal::distanceGoal (const ompl::base::State *st) const {
        Eigen::VectorXd q(ndof_);
        stateOmplToEigen(st, q, ndof_);
        KDL::Frame T_W_G;
        kin_model_->calculateFk(T_W_G, effector_name_, q);
        KDL::Twist diff = KDL::diff(T_W_G, T_W_G_dest_, 1.0);
        return diff.vel.Norm() + diff.rot.Norm();
}

