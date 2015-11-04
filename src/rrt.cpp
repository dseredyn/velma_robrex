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

#include "rrt.h"

#include "planer_utils/random_uniform.h"

    RRT::RRT(int ndof,
            boost::function<bool(const KDL::Frame &x)> collision_func,
            boost::function<void(KDL::Frame &sample)> sampleSpace_func,
            double collision_check_step, double steer_dist, double near_dist,
            const boost::shared_ptr<KinematicModel> &kin_model,
            const std::string &effector_name,
            boost::shared_ptr<DynamicsSimulatorHandPose> &sim) :
        ndof_(ndof),
        collision_func_(collision_func),
        sampleSpace_func_(sampleSpace_func),
        collision_check_step_(collision_check_step),
        steer_dist_(steer_dist),
        near_dist_(near_dist),
        kin_model_(kin_model),
        effector_name_(effector_name),
        sim_(sim)
    {
    }

    bool RRT::isPoseValid(const KDL::Frame &x) const {
        return !collision_func_(x);
    }

    void RRT::sampleSpace(KDL::Frame &sample) const {
        sampleSpace_func_(sample);
    }

    bool RRT::sampleFree(KDL::Frame &sample_free) const {
        KDL::Frame x;
        for (int i=0; i < 100; i++) {
            sampleSpace(x);
            if (isPoseValid(x)) {
                sample_free = x;
                return true;
            }
        }
        return false;
    }

    int RRT::nearest(const KDL::Frame &x, bool to_goal) const {
        double min_dist = -1.0;
        int min_idx = -1;
        for (std::map<int, RRTState >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            if (to_goal && v_it->second.ignore_to_goal_) {
                continue;
            }

            KDL::Twist diff( KDL::diff(v_it->second.T_B_E_, x, 1.0) );
            double dist = diff.vel.Norm() + diff.rot.Norm();
            if (min_idx < 0 || dist < min_dist) {
                min_dist = dist;
                min_idx = v_it->first;
            }
        }
        return min_idx;
    }

    void RRT::steer(const KDL::Frame &x_from, const KDL::Frame &x_to, double steer_dist_lin, double steer_dist_rot, KDL::Frame &x) const {
        KDL::Twist diff( KDL::diff(x_from, x_to, 1.0) );

        double dist_lin = diff.vel.Norm();

        if (dist_lin > steer_dist_lin) {
            diff.vel = steer_dist_lin * diff.vel / dist_lin;
            diff.rot = steer_dist_lin * diff.rot / dist_lin;
        }

        double dist_rot = diff.rot.Norm();
        if (dist_rot > steer_dist_rot) {
            diff.vel = steer_dist_rot * diff.vel / dist_rot;
            diff.rot = steer_dist_rot * diff.rot / dist_rot;
        }

        x = KDL::addDelta(x_from, diff, 1.0);
    }

    bool RRT::collisionFree(const Eigen::VectorXd &q_from, const Eigen::VectorXd &dq_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, Eigen::VectorXd &dq_to, KDL::Frame &x_to_out,
                            std::list<KDL::Frame > *path_x, std::list<Eigen::VectorXd > *path_q) const {
        path_x->clear();
        path_q->clear();

        Eigen::VectorXd q(ndof_), dq(ndof_), ddq(ndof_);
        ddq.setZero();
        sim_->setTarget(x_to);
        sim_->setState(q_from, dq_from, ddq);
        KDL::Twist diff_target = KDL::diff(x_from, x_to, 1.0);

        for (int loop_counter = 0; loop_counter < 3000; loop_counter++) {
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
            if (goal_diff.vel.Norm() < 0.01 && goal_diff.rot.Norm() < 5.0/180.0*3.1415) {
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

    double RRT::costLine(const KDL::Frame &x1, const KDL::Frame &x2) const {
        KDL::Twist diff( KDL::diff(x1, x2, 1.0) );
        return diff.vel.Norm() + diff.rot.Norm();
    }

    double RRT::costLine(int x1_idx, int x2_idx) const {
        return costLine(V_.find(x1_idx)->second.T_B_E_, V_.find(x2_idx)->second.T_B_E_);
    }


    double RRT::cost(int q_idx) const {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            return 0.0;
        }
        int q_parent_idx = e_it->second;
        return costLine(q_idx, q_parent_idx) + cost(q_parent_idx);
    }

    void RRT::getPath(int q_idx, std::list<int > &path) const {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            path.clear();
            path.push_back(q_idx);
        }
        else {
            int q_parent_idx = e_it->second;
            getPath(q_parent_idx, path);
            path.push_back(q_idx);
        }
    }

    void RRT::plan(const Eigen::VectorXd &q_start, const KDL::Frame &x_goal, double goal_tolerance, std::list<KDL::Frame > *path_x, std::list<Eigen::VectorXd > *path_q, MarkerPublisher &markers_pub) {
        V_.clear();
        E_.clear();
        if (path_x != NULL) {
            path_x->clear();
        }
        if (path_q != NULL) {
            path_q->clear();
        }

        int q_new_idx = 0;

        RRTState state_start;
        kin_model_->calculateFk(state_start.T_B_E_, effector_name_, q_start);
        state_start.q_ = q_start;
        state_start.dq_.resize(ndof_);
        state_start.dq_.setZero();
        V_[0] = state_start;

        bool goal_found = false;

        int m_id = 0;
        for (int step = 0; step < 100; step++) {
            bool sample_goal = false;//randomUniform(0,1) < 0.05;
            KDL::Frame x_rand;

            if (sample_goal) {
                x_rand = x_goal;
            }
            else {
                if (!sampleFree(x_rand)) {
                    std::cout << "ERROR: RRT::plan: could not sample free space" << std::endl;
                    return;
                }
            }

            // get the closest pose
            int x_nearest_idx = nearest(x_rand, sample_goal);
            if (x_nearest_idx < 0) {
                continue;
            }
            RRTState &state_nearest( V_.find(x_nearest_idx)->second );
//            KDL::Frame x_nearest = state_nearest.T_B_E_;
            KDL::Frame x_new;
            // get the new pose
//            steer(state_nearest.T_B_E_, x_rand, 1.0, 90.0/180.0*3.1415, x_new);
            steer(state_nearest.T_B_E_, x_rand, 0.5, 30.0/180.0*3.1415, x_new);

            bool is_goal_sample = false;
            KDL::Twist goal_diff( KDL::diff(x_new, x_goal, 1.0) );
            if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                is_goal_sample = true;
            }

//            std::cout << state_nearest.T_B_E_.p[0] << " " << state_nearest.T_B_E_.p[1] << " " << state_nearest.T_B_E_.p[2] << std::endl;
//            std::cout << x_new.p[0] << " " << x_new.p[1] << " " << x_new.p[2] << std::endl;

            Eigen::VectorXd q_new(ndof_), dq_new(ndof_);
            bool added_new = false;
            KDL::Frame T_B_E;
            std::list<KDL::Frame > sub_path_x;
            std::list<Eigen::VectorXd > sub_path_q;
            bool col_free = collisionFree(state_nearest.q_, state_nearest.dq_, state_nearest.T_B_E_, x_new, 0, q_new, dq_new, T_B_E, &sub_path_x, &sub_path_q);
/*                        std::list<KDL::Frame >::const_iterator x_it;
                        std::list<Eigen::VectorXd >::const_iterator q_it;
                        for (x_it = sub_path_x.begin(), q_it = sub_path_q.begin(); x_it != sub_path_x.end(); x_it++, q_it++) {
                            added_new = true;
                            RRTState state_new;
                            state_new.T_B_E_ = (*x_it);
                            state_new.q_ = (*q_it);
                            state_new.ignore_to_goal_ = false;
                            q_new_idx++;
                            V_[q_new_idx] = state_new;
                            E_[q_new_idx] = x_nearest_idx;
                            m_id = markers_pub.addVectorMarker(q_new_idx, state_nearest.T_B_E_.p, state_new.T_B_E_.p, 0, 0.7, 0, 0.5, 0.01, "world");
                            x_nearest_idx = q_new_idx;
                            x_nearest = state_new.T_B_E_;
                        }
*/
            if (col_free) {

                        added_new = true;
                        RRTState state_new;
                        state_new.T_B_E_ = T_B_E;
                        state_new.q_ = q_new;
                        state_new.dq_ = dq_new;
                        state_new.ignore_to_goal_ = false;
                        q_new_idx++;
                        V_[q_new_idx] = state_new;
                        E_[q_new_idx] = x_nearest_idx;
                        m_id = markers_pub.addVectorMarker(q_new_idx, state_nearest.T_B_E_.p, T_B_E.p, 0, 0.7, 0, 0.5, 0.01, "world");

                        KDL::Twist goal_diff( KDL::diff(T_B_E, x_goal, 1.0) );
                        if (goal_diff.vel.Norm() < 0.06 && goal_diff.rot.Norm() < 20.0/180.0*3.1415) {
                            goal_found = true;
//                            std::cout << "goal found" << std::endl;
                        }

                        if (collisionFree(state_new.q_, state_new.dq_, state_new.T_B_E_, x_goal, 0, q_new, dq_new, T_B_E, &sub_path_x, &sub_path_q)) {
                            goal_found = true;
                            RRTState state_new;
                            state_new.T_B_E_ = T_B_E;
                            state_new.q_ = q_new;
                            state_new.dq_ = dq_new;
                            state_new.ignore_to_goal_ = false;
                            q_new_idx++;
                            V_[q_new_idx] = state_new;
                            E_[q_new_idx] = q_new_idx-1;
                            m_id = markers_pub.addVectorMarker(q_new_idx, V_[q_new_idx-1].T_B_E_.p, V_[q_new_idx].T_B_E_.p, 0, 1, 0, 0.5, 0.01, "world");
                        }
            }
            else {
/*
                // this node was the goal node
                if (is_goal_sample && x_nearest_idx > 0) {
                    std::set<int> nodes_to_remove;
                    nodes_to_remove.insert(x_nearest_idx);
                    int removed_nodes_count = 0;

                    while (nodes_to_remove.size() > 0) {
                            std::set<int> orphan_nodes;
                            for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
                                if (nodes_to_remove.find(e_it->second) != nodes_to_remove.end()) {
                                    orphan_nodes.insert(e_it->first);
                                }
                            }

                            for (std::set<int>::const_iterator it = nodes_to_remove.begin(); it != nodes_to_remove.end(); it++) {
                                V_.erase( (*it) );
                                E_.erase( (*it) );
                                markers_pub.addEraseMarkers( (*it), (*it)+1);
                                removed_nodes_count++;
                            }
                            nodes_to_remove = orphan_nodes;
                    }
//                    std::cout << "removed nodes: " << removed_nodes_count << std::endl;
                }
*/
/*                if (is_goal_sample) {
                    // mark the nearest node
                    state_nearest.ignore_to_goal_ = true;
                    std::map<int, int >::const_iterator e_it = E_.find(x_nearest_idx);
                    if (e_it != E_.end()) {
                        m_id = markers_pub.addVectorMarker(x_nearest_idx, V_[e_it->second].T_B_E_.p, x_nearest.p, 1, 0, 0, 0.5, 0.01, "world");
                    }
                }
*/
/*
                if (is_goal_sample) {
                    std::list<int> nodes_to_remove_list;
                    getPath(x_nearest_idx, nodes_to_remove_list);
                    if (nodes_to_remove_list.size() > 1) {
                        nodes_to_remove_list.pop_front();
                        std::set<int> nodes_to_remove;

                        int removed_nodes_count = 0;
                        for (std::list<int>::const_iterator it = nodes_to_remove_list.begin(); it != nodes_to_remove_list.end(); it++) {
                            nodes_to_remove.insert( (*it) );
                        }

                        while (nodes_to_remove.size() > 0) {
                            std::set<int> orphan_nodes;
                            for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
                                if (nodes_to_remove.find(e_it->second) != nodes_to_remove.end()) {
                                    orphan_nodes.insert(e_it->first);
                                }
                            }

                            for (std::set<int>::const_iterator it = nodes_to_remove.begin(); it != nodes_to_remove.end(); it++) {
                                V_.erase( (*it) );
                                E_.erase( (*it) );
                                markers_pub.addEraseMarkers( (*it), (*it)+1);
                                removed_nodes_count++;
                            }
                            nodes_to_remove = orphan_nodes;
                        }
                        std::cout << "removed nodes: " << removed_nodes_count << std::endl;
                    }
                }
//*/
            }

            if (added_new) {
//                std::cout << "step " << step << "  nearest_idx " << x_nearest_idx << std::endl;
            }

            markers_pub.publish();
            ros::spinOnce();
//            getchar();
            if (goal_found) {
                break;
            }
        }

        if (goal_found) {
            std::list<int > idx_path;
            getPath(q_new_idx, idx_path);
            if (path_x != NULL) {
                for (std::list<int >::const_iterator p_it = idx_path.begin(); p_it != idx_path.end(); p_it++) {
                    const RRTState &current = V_.find(*p_it)->second;
                    path_x->push_back(current.T_B_E_);
                }
            }
            if (path_q != NULL) {
                // execute the planned path and save the joint trajectory
                Eigen::VectorXd dq( Eigen::VectorXd::Zero(ndof_) );
                Eigen::VectorXd ddq( Eigen::VectorXd::Zero(ndof_) );
                sim_->setState(q_start, dq, ddq);
                for (std::list<int >::const_iterator p_it = idx_path.begin(); p_it != idx_path.end(); p_it++) {
                    const RRTState &current = V_.find(*p_it)->second;
                    sim_->setTarget( current.T_B_E_ );
                    for (int loop_counter = 0; loop_counter < 3000; loop_counter++) {
                        Eigen::VectorXd q(ndof_), dq(ndof_), ddq(ndof_);
                        sim_->getState(q, dq, ddq);

                        bool added_point = false;
                        if (path_q->empty()) {
                            added_point = true;
                            path_q->push_back(q);
                        }
                        else {
                            if ( (path_q->back() - q).norm() > 10.0/180.0*3.1415 ) {
                                added_point = true;
                                path_q->push_back(q);
                            }
                        }

                        KDL::Frame r_HAND_current;
                        kin_model_->calculateFk(r_HAND_current, effector_name_, q);

                        KDL::Twist goal_diff( KDL::diff(r_HAND_current, current.T_B_E_, 1.0) );
                        if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                            if (!added_point) {
                                path_q->push_back(q);
                            }
                            break;
                        }
                        sim_->oneStep();
                    }
                }
            }
        }
    }

    int RRT::addTreeMarker(MarkerPublisher &markers_pub, int m_id) const {
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

