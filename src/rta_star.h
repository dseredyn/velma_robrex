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

#ifndef RTA_STAR_H__
#define RTA_STAR_H__

#include "Eigen/Dense"

#include "planer_utils/marker_publisher.h"
#include "kin_model/kin_model.h"
#include "planer_utils/simulator.h"

class RTAStarState {
public:
    RTAStarState(int nodes);
    KDL::Frame T_B_E_;
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    std::vector<int > neighbour_nodes_;
    std::vector<bool > simulated_nodes_;
    int parent_idx_;
    int parent_tr_idx_;
    double h_value_;
};

class RTAStar {
public:

    RTAStar(int ndof,
            boost::function<bool(const KDL::Frame &x)> collision_func,
            double collision_check_step, double steer_dist, double near_dist,
            const boost::shared_ptr<KinematicModel> &kin_model,
            const std::string &effector_name,
            boost::shared_ptr<DynamicsSimulatorHandPose> &sim);

    bool isPoseValid(const KDL::Frame &x) const;

    bool collisionFree(const Eigen::VectorXd &q_from, const Eigen::VectorXd &dq_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, Eigen::VectorXd &dq_to, KDL::Frame &x_to_out,
                            std::list<KDL::Frame > *path_x, std::list<Eigen::VectorXd > *path_q) const;

//    bool collisionFree(const Eigen::VectorXd &q_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, KDL::Frame &x_to_out,
//                        std::list<KDL::Frame > *path_x, std::list<Eigen::VectorXd > *path_q) const;

    void plan(const Eigen::VectorXd &q_start, const KDL::Frame &x_goal, std::list<Eigen::VectorXd > &path_q, MarkerPublisher &markers_pub);

    int addTreeMarker(MarkerPublisher &markers_pub, int m_id) const;

    int checkIfStateExists(int excluded_index, const RTAStarState &state) const;

    double getCostLine(const KDL::Frame &x1, const KDL::Frame &x2) const;

//    bool expand_graph(int node_idx, double parent_goal_min_cost, int parent_node_idx, int parent_tr_idx, MarkerPublisher &markers_pub);

    void setGoal(const KDL::Frame &g);

    double getCostH(const KDL::Frame &x) const;

    double lookahead(const KDL::Frame &T_B_E, int depth) const;

protected:

    int q_new_idx_;
    std::vector<KDL::Frame > transform_delta_vec_;
    std::map<int, int> inverse_transformation_map_;
    KDL::Frame goal_;

    boost::function<bool(const KDL::Frame &x)> collision_func_;
    boost::function<void(KDL::Frame &sample)> sampleSpace_func_;
    std::map<int, RTAStarState > V_;
    std::map<int, int > E_;
    double collision_check_step_;
    int ndof_;
    double steer_dist_;
    double near_dist_;
    const boost::shared_ptr<KinematicModel> &kin_model_;
    std::string effector_name_;
    boost::shared_ptr<DynamicsSimulatorHandPose> &sim_;
};

#endif  // RTA_STAR_H__

