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

#include "rta_star.h"

#include "planer_utils/random_uniform.h"

    RTAStarState::RTAStarState(int nodes) {
        neighbour_nodes_.resize(nodes, -1);
        simulated_nodes_.resize(nodes, false);
    }

    RTAStar::RTAStar(int ndof,
            boost::function<bool(const KDL::Frame &x)> collision_func,
            double collision_check_step, double steer_dist, double near_dist,
            const boost::shared_ptr<KinematicModel> &kin_model,
            const std::string &effector_name,
            boost::shared_ptr<DynamicsSimulatorHandPose> &sim) :
        ndof_(ndof),
        collision_func_(collision_func),
        collision_check_step_(collision_check_step),
        steer_dist_(steer_dist),
        near_dist_(near_dist),
        kin_model_(kin_model),
        effector_name_(effector_name),
        sim_(sim)
    {
        transform_delta_vec_.push_back( KDL::Frame(KDL::Vector(0.08, 0.0, 0.0)) );
        transform_delta_vec_.push_back( KDL::Frame(KDL::Vector(-0.08, 0.0, 0.0)) );
        transform_delta_vec_.push_back( KDL::Frame(KDL::Vector(0.0, 0.08, 0.0)) );
        transform_delta_vec_.push_back( KDL::Frame(KDL::Vector(0.0, -0.08, 0.0)) );
        transform_delta_vec_.push_back( KDL::Frame(KDL::Vector(0.0, 0.0, 0.08)) );
        transform_delta_vec_.push_back( KDL::Frame(KDL::Vector(0.0, 0.0, -0.08)) );

        inverse_transformation_map_[-1] = -1;
        inverse_transformation_map_[0] = 1;
        inverse_transformation_map_[1] = 0;
        inverse_transformation_map_[2] = 3;
        inverse_transformation_map_[3] = 2;
        inverse_transformation_map_[4] = 5;
        inverse_transformation_map_[5] = 4;
    }

    bool RTAStar::isPoseValid(const KDL::Frame &x) const {
        return !collision_func_(x);
    }

    bool RTAStar::collisionFree(const Eigen::VectorXd &q_from, const Eigen::VectorXd &dq_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, Eigen::VectorXd &dq_to, KDL::Frame &x_to_out,
                            std::list<KDL::Frame > *path_x, std::list<Eigen::VectorXd > *path_q) const {
        path_x->clear();
        path_q->clear();

        Eigen::VectorXd q(ndof_), dq(ndof_), ddq(ndof_);
        ddq.setZero();
        sim_->setTarget(x_to);
        sim_->setState(q_from, dq_from, ddq);
        KDL::Twist diff_target = KDL::diff(x_from, x_to, 1.0);

        for (int loop_counter = 0; loop_counter < 1500; loop_counter++) {
//            Eigen::VectorXd q(ndof_), dq(ndof_), ddq(ndof_);
            sim_->getState(q, dq, ddq);

            KDL::Frame r_HAND_current;
            kin_model_->calculateFk(r_HAND_current, effector_name_, q);

            KDL::Twist prev_diff;
            if (path_x->empty()) {
                prev_diff = KDL::diff(x_from, r_HAND_current, 1.0);
            }
            else {
                prev_diff = KDL::diff(path_x->back(), r_HAND_current, 1.0);
            }

            if (prev_diff.vel.Norm() > 0.1 || prev_diff.rot.Norm() > 30.0/180.0*3.1415) {
                path_x->push_back(r_HAND_current);
                path_q->push_back(q);
            }

            KDL::Twist goal_diff( KDL::diff(r_HAND_current, x_to, 1.0) );
            if (goal_diff.vel.Norm() < 0.01) {// && goal_diff.rot.Norm() < 5.0/180.0*3.1415) {
                q_to = q;
                dq_to = dq;
                x_to_out = r_HAND_current;
                return true;
            }

            sim_->oneStep();
            if (sim_->inCollision()) {
                return false;
            }
        }
        return false;
    }

    void RTAStar::setGoal(const KDL::Frame &g) {
        goal_ = g;
    }

    double RTAStar::getCostH(const KDL::Frame &x) const {
        return (goal_.p - x.p).Norm();
    }

    double RTAStar::getCostLine(const KDL::Frame &x1, const KDL::Frame &x2) const {
        return (x1.p - x2.p).Norm();
    }

    double RTAStar::lookahead(const KDL::Frame &T_B_E, int depth) const {
        int state_idx = -1;//checkIfStateExists(T_B_E);
        double min_cost = -1;
        if (state_idx == -1) {
            if (depth == 1) {
                for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                    double costF = getCostLine(KDL::Frame(), transform_delta_vec_[tr_idx]) + getCostH(transform_delta_vec_[tr_idx] * T_B_E);
//                    std::cout << "RTAStar::lookahead " << costF << " " << getCostLine(KDL::Frame(), transform_delta_vec_[tr_idx]) << " " << getCostH(transform_delta_vec_[tr_idx] * T_B_E) << std::endl;
                    if (min_cost < 0 || costF < min_cost) {
                        min_cost = costF;
                    }
                }
            }
            else {
                for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                    double costF = getCostLine(KDL::Frame(), transform_delta_vec_[tr_idx]) + lookahead(transform_delta_vec_[tr_idx] * T_B_E, depth - 1);
                    if (min_cost < 0 || costF < min_cost) {
                        min_cost = costF;
                    }
                }
            }
        }
        else {
            std::map<int, RTAStarState >::const_iterator v_it = V_.find(state_idx);
            min_cost = getCostLine(T_B_E, v_it->second.T_B_E_) + v_it->second.h_value_;
        }
        return min_cost;
    }

//    int RTAStar::checkIfStateExists(const KDL::Frame &T_B_E) const {
    int RTAStar::checkIfStateExists(int excluded_index, const RTAStarState &state) const {
        for (std::map<int, RTAStarState >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            if (v_it->first == excluded_index) {
                continue;
            }
            if (v_it->second.q_.innerSize() < ndof_) {
                continue;
            }
            if ( (state.q_ - v_it->second.q_).norm() < 5.0/180.0*3.11415) {
//            KDL::Twist diff = KDL::diff(T_B_E, v_it->second.T_B_E_);
//            if (diff.vel.Norm() < 0.04) {
                return v_it->first;
            }
        }
        return -1;
    }

    void RTAStar::plan(const Eigen::VectorXd &q_start, const KDL::Frame &x_goal, std::list<Eigen::VectorXd > &path_q, MarkerPublisher &markers_pub) {
        path_q.clear();
        V_.clear();
        E_.clear();
        setGoal(x_goal);

        std::vector<double > tmp_vec;
        tmp_vec.resize(transform_delta_vec_.size(), 0.0);
        int q_new_idx_ = 0;

        {
            RTAStarState state_start( transform_delta_vec_.size() );
            state_start.h_value_ = 0.0;
            state_start.q_ = q_start;
            state_start.dq_.resize(ndof_);
            state_start.dq_.setZero();
            state_start.parent_idx_ = -1;
//            state_start.T_B_E_ = KDL::Frame(KDL::Vector(q_start(0), q_start(1), q_start(2)));
            kin_model_->calculateFk(state_start.T_B_E_, effector_name_, q_start);
            V_.insert(std::make_pair(0, state_start));
        }

        int current_node_idx = 0;
        while (true) {
            std::map<int, RTAStarState >::iterator v_it = V_.find(current_node_idx);
            if (v_it == V_.end()) {
                std::cout << "ERROR: RTAStar::plan v_it == V_.end() " << current_node_idx << std::endl;
                return;
            }

            if ((v_it->second.T_B_E_.p - x_goal.p).Norm() < 0.08 && v_it->second.q_.innerSize() == ndof_) {
                std::cout << "goal reached" << std::endl;
                while (true) {
                    path_q.push_front(v_it->second.q_);
                    current_node_idx = v_it->second.parent_idx_;
                    if (current_node_idx < 0) {
                        break;
                    }
                    v_it = V_.find(current_node_idx);
                    if (v_it == V_.end()) {
                        std::cout << "ERROR: v_it == V_.end() " << current_node_idx << std::endl;
                    }
                }
                return;
            }

            // From a given current state, the neighbouring states are generated
            for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                // the neighbour state was not yet visited from the current node
                if (v_it->second.neighbour_nodes_[tr_idx] == -1) {
                    // create a new state and add it to the graph
                    RTAStarState new_state( transform_delta_vec_.size() );
                    new_state.T_B_E_ = transform_delta_vec_[tr_idx] * v_it->second.T_B_E_;

                    // check if this neighbour state already exists
                    int state_idx = -1;//checkIfStateExists(new_state.T_B_E_);
                    if (state_idx == -1) {
                        // create a new state
                        q_new_idx_++;
                        new_state.h_value_ = getCostH(new_state.T_B_E_);
                        new_state.parent_idx_ = v_it->first;
                        new_state.parent_tr_idx_ = tr_idx;
                        v_it->second.neighbour_nodes_[tr_idx] = q_new_idx_;
                        V_.insert(std::make_pair(q_new_idx_, new_state));

                        markers_pub.addSinglePointMarker(q_new_idx_, new_state.T_B_E_.p, 0, 1, 0, 1, 0.05, "world");
                    }
                    else {
                        // add existing state to neighbour list
                        v_it->second.neighbour_nodes_[tr_idx] = state_idx;
                    }
                }
            }

            // The heuristic function, augmented by lookahead search, is applied to each,
            // and then the cost of the edge to each neighbouring state is added to this value,
            // resulting in an f value for each neighbour of the current state.
            bool validTransitionExists = false;
            for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                int neighbour_idx = v_it->second.neighbour_nodes_[tr_idx];
                if (neighbour_idx >= 0) {
                    std::map<int, RTAStarState >::const_iterator n_it = V_.find(neighbour_idx);
                    if (n_it == V_.end()) {
                        std::cout << "ERROR: RTAStar::plan n_it == V_.end() " << neighbour_idx << std::endl;
                        return;
                    }

                    // calculate the lookahead heuristics for each neighbour and add the cost from current state to neighbour
                    double cost_h = lookahead(n_it->second.T_B_E_, 5);
                    tmp_vec[tr_idx] = cost_h + getCostLine(v_it->second.T_B_E_, n_it->second.T_B_E_);
                    std::cout << "    cost_h " << tr_idx << " " << cost_h << std::endl;
                    validTransitionExists = true;
                }
                else {
                    tmp_vec[tr_idx] = 10000.0;
                }
            }

            if (!validTransitionExists) {
                // remove the current node
                std::cout << "   no possible transition " << v_it->first << std::endl;
                std::map<int, RTAStarState >::iterator parent_it = V_.find(v_it->second.parent_idx_);
                if (parent_it == V_.end()) {
                    std::cout << "no parent node" << std::endl;
                    return;
                }

                parent_it->second.neighbour_nodes_[v_it->second.parent_tr_idx_] = -2;

                markers_pub.addSinglePointMarker(v_it->first, v_it->second.T_B_E_.p, 0, 0, 0, 1, 0.05, "world");
                V_.erase(current_node_idx);
                current_node_idx = parent_it->first;
                markers_pub.publish();
                ros::spinOnce();
                continue;
            }

            // The node with the minimum f value is chosen for the new current state and a move to that state
            // is executed. At the same time, the next best f value is stored at the previous current state.

            // get the two smallest values
            double min1 = -1.0, min2 = -1.0;
            int min1_idx = -1;
            for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                if (min1 < 0 || min1 > tmp_vec[tr_idx]) {
                    min1 = tmp_vec[tr_idx];
                    min1_idx = tr_idx;
                }
            }

            for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                if (tr_idx == min1_idx) {
                    continue;
                }
                if (min2 < 0 || min2 > tmp_vec[tr_idx]) {
                    min2 = tmp_vec[tr_idx];
                }
            }

            std::cout << "current_node_idx " << current_node_idx << "  min: " << min1 << " " << min2 << std::endl;

            // execute the move
            int next_node_idx = v_it->second.neighbour_nodes_[min1_idx];
            if (next_node_idx < 0) {
                std::cout << "ERROR: next_node_idx < 0: " << next_node_idx << std::endl;
                return;
            }
            std::map<int, RTAStarState >::iterator n_it = V_.find(next_node_idx);
//            std::cout << n_it->second.T_B_E_.p[0] << " " << n_it->second.T_B_E_.p[1] << " " << n_it->second.T_B_E_.p[2] << std::endl;
            // check if the next state is possible

            if (!v_it->second.simulated_nodes_[min1_idx]) {
                // simulate
                n_it->second.q_.resize(ndof_);
                n_it->second.dq_.resize(ndof_);
                KDL::Frame nT_B_E;
                std::list<KDL::Frame > path_x;
                std::list<Eigen::VectorXd > path_q;
                KDL::Frame T_B_E(n_it->second.T_B_E_);
                bool col_free = collisionFree(v_it->second.q_, v_it->second.dq_, v_it->second.T_B_E_, n_it->second.T_B_E_, 0, n_it->second.q_, n_it->second.dq_, nT_B_E, &path_x, &path_q);

                int similar_state = checkIfStateExists(n_it->first, n_it->second);
                if (!col_free || similar_state >=0) {
                    n_it->second.h_value_ = 100.0;
                    if (!col_free) {
                        std::cout << "   invalid state change, removing node " << next_node_idx << std::endl;
                    }
                    else {
                        std::cout << "   joining states " << next_node_idx << " to " << similar_state << std::endl;
                    }
                    V_.erase(next_node_idx);
                    v_it->second.neighbour_nodes_[min1_idx] = -2;
                    markers_pub.addSinglePointMarker(n_it->first, n_it->second.T_B_E_.p, 0, 0, 0, 1, 0.05, "world");
                    markers_pub.publish();
                    ros::spinOnce();
                    continue;
                }

                v_it->second.simulated_nodes_[min1_idx] = true;
            }

/*            int similar_state = checkIfStateExists(n_it->first, n_it->second);
            if (similar_state >= 0) {
                // TODO: possible infinite loop!
                // join states
                v_it->second.neighbour_nodes_[min1_idx] = similar_state;
                V_.erase(next_node_idx);
                std::cout << "joining states: " << next_node_idx << " to " << similar_state << std::endl;
                next_node_idx = similar_state;
            }
*/
/*
                if (!col_free) {isPoseValid(n_it->second.T_B_E_)) {
                    n_it->second.h_value_ = 10.0;
                    markers_pub.addSinglePointMarker(n_it->first, n_it->second.T_B_E_.p, 0, 0, 0, 1, 0.05, "world");
                    markers_pub.publish();
                    ros::spinOnce();
                    std::cout << "   invalid pose" << std::endl;
                    continue;
                }
*/
            double min_h = 10000.0;
            int min_idx = -1;
            for (std::map<int, RTAStarState >::const_iterator v2_it = V_.begin(); v2_it != V_.end(); v2_it++) {
                if (v2_it->second.h_value_ < min_h && v2_it->second.q_.innerSize() == ndof_) {
                    min_h = v2_it->second.h_value_;
                    min_idx = v2_it->first;
                }
//                markers_pub.addVectorMarker(v2_it->first+6000, v2_it->second.T_B_E_.p + KDL::Vector(0,0,0.05), v2_it->second.T_B_E_.p + KDL::Vector(0,0,0.05 + v2_it->second.h_value_*0.1), 0, 0.7, 0, 0.5, 0.01, "world");
            }

            next_node_idx = min_idx;

            markers_pub.addSinglePointMarker(v_it->first, v_it->second.T_B_E_.p, 1, 0, 0, 1, 0.05, "world");
            markers_pub.addSinglePointMarker(n_it->first, n_it->second.T_B_E_.p, 0, 0, 1, 1, 0.05, "world");
            markers_pub.publish();
            ros::spinOnce();
//            getchar();

            markers_pub.addSinglePointMarker(v_it->first, v_it->second.T_B_E_.p, 0, 1, 0, 1, 0.05, "world");
            markers_pub.addSinglePointMarker(n_it->first, n_it->second.T_B_E_.p, 0, 1, 0, 1, 0.05, "world");

            // save the second minimum value
            v_it->second.h_value_ = min2;

            std::cout << "changing state to " << next_node_idx << std::endl;
            // go to the next state
            current_node_idx = next_node_idx;
//            getchar();
        }
    }
/*
    bool RTAStar::expand_graph(int current_node_idx, double parent_goal_min_cost, int parent_node_idx, int parent_tr_idx, MarkerPublisher &markers_pub) {
        std::map<int, RTAStarState >::iterator v_it = V_.find(current_node_idx);
        std::map<int, RTAStarState >::iterator vp_it = V_.find(parent_node_idx);

        if (v_it == V_.end()) {
            std::cout << "ERROR: RTAStar::expand_graph v_it == V_.end() " << current_node_idx << std::endl;
            return true;
        }

        double cost_back_to_parent;

        if (vp_it != V_.end()) {
            cost_back_to_parent = parent_goal_min_cost + 1.0 * getCostLine(v_it->second.T_B_E_, vp_it->second.T_B_E_);
        }

        if (getCostToGoal( v_it->second.T_B_E_ ) < 0.1) {
            std::cout << "RTAStar::expand_graph reached goal" << std::endl;
            return true;
        }

        int forbidden_tr_idx = inverse_transformation_map_[parent_tr_idx];
        while (true) {
            double min_cost_current = -1.0;
            double min_cost_current2 = -1.0;
            KDL::Frame min_B_E_new;
            int min_tr_idx = -1;
            for (int tr_idx = 0; tr_idx < transform_delta_vec_.size(); tr_idx++) {
                if (v_it->second.neighbour_nodes_[tr_idx] || tr_idx == forbidden_tr_idx) {
                    continue;
                }

                KDL::Frame T_B_E_new( transform_delta_vec_[tr_idx].p + v_it->second.T_B_E_.p );
                double total_cost_goal = v_it->second.distance_from_start_ + transform_delta_vec_[tr_idx].p.Norm() + getCostToGoal( T_B_E_new );

                if (min_cost_current < 0) {
                    min_cost_current = total_cost_goal;
                    min_B_E_new = T_B_E_new;
                    min_tr_idx = tr_idx;
                }
                else if (total_cost_goal < min_cost_current) {
                    min_cost_current2 = min_cost_current;
                    min_cost_current = total_cost_goal;
                    min_B_E_new = T_B_E_new;
                    min_tr_idx = tr_idx;
                }
                else if (total_cost_goal < min_cost_current2) {
                    min_cost_current2 = total_cost_goal;
                }

                if (min_cost_current >= 0 && min_cost_current2 < 0) {
                    min_cost_current2 = total_cost_goal;
                }

            }

            double next_node_cost_from_start = v_it->second.distance_from_start_ + transform_delta_vec_[min_tr_idx].p.Norm();

            if (min_cost_current < 0) {
                // nothing to extend - go to parent
                std::cout << "RTAStar::expand_graph " << cost_back_to_parent << " " << min_cost_current << " " << current_node_idx << " " << parent_node_idx << " nothing to expand" << std::endl;
                return false;
            }


            if (vp_it != V_.end() && cost_back_to_parent <= min_cost_current) {
//            if (vp_it != V_.end() && vp_it->second.distance_from_start_ + parent_goal_min_cost <= distance_from_start + min_cost_current) {
                // it is better to go to parent
                std::cout << "RTAStar::expand_graph " << cost_back_to_parent << " " << min_cost_current << " " << current_node_idx << " " << parent_node_idx << " go to parent" << std::endl;
                return false;
            }

            std::cout << "RTAStar::expand_graph " << cost_back_to_parent << " " << min_cost_current << " " << current_node_idx << " " << parent_node_idx << " expand" << std::endl;
            std::cout << "v_it->second.distance_from_start_ " << v_it->second.distance_from_start_ << std::endl;
            markers_pub.addSinglePointMarker(q_new_idx_, min_B_E_new.p, 0, 1, 0, 1, 0.05, "world");
                markers_pub.publish();
                ros::spinOnce();

            getchar();

            // it is better to expand current node
            v_it->second.visited_nodes_[min_tr_idx] = true;

            // check if the next state is possible
            if (!isPoseValid(min_B_E_new)) {
                std::cout << "   invalid pose" << std::endl;
                continue;
            }

            RTAStarState state_new( transform_delta_vec_.size() );
            state_new.T_B_E_ = min_B_E_new;
            state_new.distance_from_start_ = next_node_cost_from_start;
            q_new_idx_++;
            V_.insert(std::make_pair(q_new_idx_, state_new));
            E_[q_new_idx_] = current_node_idx;

            if (expand_graph(q_new_idx_, min_cost_current2, current_node_idx, min_tr_idx, markers_pub)) {
                return true;
            }
        }
                    // sort the collisions by distance (ascending order)
//            std::sort(link_collisions.begin(), link_collisions.end(), self_collision::compareCollisionInfoDist);

    }
*/
/*
    void RTAStar::plan(const Eigen::VectorXd &q_start, const KDL::Frame &x_goal, double goal_tolerance, std::list<KDL::Frame > *path_x, std::list<Eigen::VectorXd > *path_q, MarkerPublisher &markers_pub) {
        V_.clear();
        E_.clear();
        if (path_x != NULL) {
            path_x->clear();
        }
        if (path_q != NULL) {
            path_q->clear();
        }

        setGoal(x_goal);

        int q_new_idx_ = 0;

        RTAStarState state_start( transform_delta_vec_.size() );
        state_start.distance_from_start_ = 0.0;
        kin_model_->calculateFk(state_start.T_B_E_, effector_name_, q_start);
//        state_start.q_ = q_start;
//        state_start.dq_.resize(ndof_);
//        state_start.dq_.setZero();
        V_.insert(std::make_pair(0, state_start));

        int current_node_idx = 0;
        expand_graph(0, -1.0, -1, -1, markers_pub);

    }
*/
    int RTAStar::addTreeMarker(MarkerPublisher &markers_pub, int m_id) const {
/*        std::vector<std::pair<KDL::Vector, KDL::Vector > > vec_arr;
        for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
            const Eigen::VectorXd &x1 = V_.find(e_it->first)->second.T_B_E;
            const Eigen::VectorXd &x2 = V_.find(e_it->second)->second.T_B_E;
            KDL::Vector pos1(x1(0), x1(1), 0), pos2(x2(0), x2(1), 0);
            vec_arr.push_back( std::make_pair(pos1, pos2) );
            m_id = markers_pub.addVectorMarker(m_id, pos1, pos2, 0, 0.7, 0, 0.5, 0.01, "base");
        }

        const Eigen::VectorXd &xs = V_.find(0)->second;
        KDL::Vector pos(xs(0), xs(1), 0);
        m_id = markers_pub.addSinglePointMarker(m_id, pos, 0, 1, 0, 1, 0.05, "base");
*/
        return m_id;
    }

