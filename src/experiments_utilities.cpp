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

#include "experiments_utilities.h"

#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"

#define IKFAST_HAS_LIBRARY
#include "ikfast.h"

void TestScenario::addNode(const KDL::Frame &T_W_G_dest, const Eigen::VectorXd &q_init, bool use_prev_q) {
    TestNode n;
    n.T_W_G_dest_ = T_W_G_dest;
    n.q_init_ = q_init;
    n.use_prev_q_ = use_prev_q;

    nodes_list_.push_back( n );
}

void TestScenario::addNode(const KDL::Frame &T_W_G_dest, const double q_init[], int ndof, bool use_prev_q) {
    Eigen::VectorXd q(ndof);
    for (int q_idx = 0; q_idx < ndof; q_idx++) {
        q(q_idx) = q_init[q_idx];
    }
    TestNode n;
    n.T_W_G_dest_ = T_W_G_dest;
    n.q_init_ = q;
    n.use_prev_q_ = use_prev_q;

    nodes_list_.push_back( n );
}

int TestScenario::getNodeId() const {
    return nodeId_;
}

int TestScenario::getNodes() const {
    return nodes_list_.size();
}

void TestScenario::startTest() {
    nodes_it_ = nodes_list_.begin();
    nodeId_ = 0;
}

void TestScenario::nextNode() {
    nodes_it_++;
    nodeId_++;
}

bool TestScenario::isFinished() const {
    return nodes_it_ == nodes_list_.end();
}

const KDL::Frame& TestScenario::getDestFrame() const {
    return nodes_it_->T_W_G_dest_;
}

const Eigen::VectorXd& TestScenario::getInitQ() const {
    return nodes_it_->q_init_;
}

bool TestScenario::usePrevQ() const {
    return nodes_it_->use_prev_q_;
}

void TestResults::addResult(const std::string &planner_name, int nodeId, bool solutionFound, bool inCollision, double planningTime, double length) {
    SingleResult r;
    r.solutionFound_ = solutionFound;
    r.inCollision_ = inCollision;
    r.planningTime_ = planningTime;
    r.length_ = length;
    r.nodeId_ = nodeId;

    results_[planner_name].push_back(r);
}
/*
double TestResults::getTotalMeanPathLength(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
    double meanPathLength = 0.0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        meanPathLength += lengthVec[i];
    }
    return meanPathLength / static_cast<double>(solutionFoundVec.size());
}
*/
double TestResults::getTotalMeanPlanningTime(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
    double meanPlanningTime = 0.0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        meanPlanningTime += planningTimeVec[i];
    }
    return meanPlanningTime / static_cast<double>(solutionFoundVec.size());
}

double TestResults::getTotalPlanningTimeVariance(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
    double meanPlanningTime = getTotalMeanPlanningTime(planner_name, nodeId);
    double variance = 0.0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        variance += (meanPlanningTime - planningTimeVec[i]) * (meanPlanningTime - planningTimeVec[i]);
    }
    return variance;
}

double TestResults::getSuccessMeanPathLength(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
    int success_count = 0;
    double meanPathLength = 0.0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        if (solutionFoundVec[i] && !inCollisionVec[i]) {
            success_count++;
            meanPathLength += lengthVec[i];
        }
    }
    return meanPathLength / static_cast<double>(success_count);
}

double TestResults::getSuccessPathLengthVariance(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);

    double meanPathLength = getSuccessMeanPathLength(planner_name, nodeId);
    double variance = 0.0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        if (solutionFoundVec[i] && !inCollisionVec[i]) {
            variance += (meanPathLength - lengthVec[i]) * (meanPathLength - lengthVec[i]);
        }
    }
    return variance;
}

/*
double TestResults::getSuccessMeanPlanningTime(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
    int success_count = 0;
    double meanPlanningTime = 0.0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        if (solutionFoundVec[i] && !inCollisionVec[i]) {
            success_count++;
            meanPlanningTime += planningTimeVec[i];
        }
    }
    return meanPlanningTime / static_cast<double>(success_count);
}
*/
double TestResults::getSuccessRate(const std::string &planner_name, int nodeId) const {
    std::vector<bool> solutionFoundVec;
    std::vector<bool> inCollisionVec;
    std::vector<double> planningTimeVec;
    std::vector<double> lengthVec;
    getTries(planner_name, nodeId, solutionFoundVec, inCollisionVec, planningTimeVec, lengthVec);
    int success_count = 0;
    for (int i = 0; i < solutionFoundVec.size(); i++) {
        if (solutionFoundVec[i] && !inCollisionVec[i]) {
            success_count++;
        }
    }
    return static_cast<double>(success_count) / static_cast<double>(solutionFoundVec.size());
}

void TestResults::getTries(const std::string &planner_name, int nodeId, std::vector<bool> &solutionFoundVec, std::vector<bool> &inCollisionVec, std::vector<double> &planningTimeVec, std::vector<double> &lengthVec) const {
    solutionFoundVec.clear();
    inCollisionVec.clear();
    planningTimeVec.clear();
    lengthVec.clear();

    std::map<std::string, std::vector<SingleResult > >::const_iterator it = results_.find( planner_name );
    if (it == results_.end()) {
        return;
    }

    for (int i = 0; i < it->second.size(); i++) {
        if (it->second[i].nodeId_ == nodeId) {
            solutionFoundVec.push_back( it->second[i].solutionFound_ );
            inCollisionVec.push_back( it->second[i].inCollision_ );
            planningTimeVec.push_back( it->second[i].planningTime_ );
            lengthVec.push_back( it->second[i].length_ );
        }
    }
}

void showMetric(const Eigen::VectorXd &q, const KDL::Vector &lower_bound, const KDL::Vector &upper_bound,
                    const boost::shared_ptr<KinematicModel> &kin_model, const boost::shared_ptr<self_collision::CollisionModel> &col_model,
                    const boost::shared_ptr<ReachabilityMap > &r_map, MarkerPublisher &markers_pub) {
        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        bool show_path = false;
/*
//        double z = 1.2;
        double y = -0.5;
        while (ros::ok()) {
            int m_id = 0;
            if (y > 0.5) {
                break;
            }
//            z += 0.025;
            y += 0.01;
//            for (double x = lower_bound(0)+0.02; x < upper_bound(0); x += 0.01) {
            for (double x = 0.7; x < 1.8; x += 0.01) {
                {
//                    double y = 0.0;
//                for (double y = lower_bound(1)+0.0125; y < upper_bound(1); y += 0.04) {
//                    for (double z = lower_bound(2)+0.02; z < upper_bound(2); z += 0.01)
                    for (double z = 1.0; z < 2.1; z += 0.01)
                    {
//                        double z = 1.4;
                        KDL::Vector pt_W(x-0.02,y-0.02,z-0.02);
                        double val = 0.0;
                        {
                            val /= 1.1;
                            KDL::Vector gr;
                            if (r_map->getGradient(pt_W, gr)) {
                                double dist = 0.0;
                                r_map->getDistance(pt_W, dist);
                                dist = dist * 0.1;

                                double cr=1.0, cg=1.0, cb=1.0;
                                if (dist > 0) {
                                    cb -= std::min(1.0, dist);
                                    dist -= 1.0;
                                }
                                if (dist > 0) {
                                    cg -= std::min(1.0, dist);
                                    dist -= 1.0;
                                }
                                if (dist > 0) {
                                    cr -= std::min(1.0, dist);
                                    dist -= 1.0;
                                }

                                m_id = markers_pub.addSinglePointMarkerCube(m_id, pt_W+KDL::Vector(0.02, 0.02, 0.02), cr, cg, cb, 0.9, 0.01, 0.0001, 0.01, "world");

//                                m_id = markers_pub.addVectorMarker(m_id, pt_W, pt_W + gr*0.020, cr, cg, cb, 1, 0.006, "world");
//                                m_id = markers_pub.addSinglePointMarker(m_id, pt_W, 1, val, val, 0.5, 0.003, "world");
                            }
                            else {
                                m_id = markers_pub.addSinglePointMarker(m_id, pt_W, 0, 0, 1, 0, 0.003, "world");
                            }
                        }
                    }
                }
            }

            if (show_path) {
                KDL::Vector rand_pt(randomUniform(lower_bound(0)-1, upper_bound(0)+1), randomUniform(lower_bound(1), upper_bound(1)), randomUniform(lower_bound(2), upper_bound(2)));
                m_id = markers_pub.addSinglePointMarker(m_id, rand_pt, 1, 0, 0, 0.5, 0.1, "world");
                int path_length = 0;
                while (true) {
                    bool use_twist = false;
                    if (use_twist) {
    //                    KDL::Twist twist = distanceMetric(KDL::Frame(rand_pt), KDL::Frame(KDL::Vector(1.05, 0.0, 1.35)), r_map);
    //                    KDL::Vector prev = rand_pt;
    //                    rand_pt = rand_pt + twist.vel * 0.02;
    //                    m_id = markers_pub.addVectorMarker(m_id, prev, rand_pt, 0, 1, 0, 0.5, 0.01, "world");
                    }
                    else {
                        KDL::Vector gr;
                        double dist = 0.0;
                        if (!r_map->getDistance(rand_pt, dist) || dist < 0.05) {
                            std::cout << "distance failed" << std::endl;
                            break;
                        }
                        std::vector<ReachabilityMap::GradientInfo > gradients;
                        gradients.resize(27);
                        r_map->getAllGradients(rand_pt, gradients);
                        for (int i=0; i<27; i++) {
                            if (gradients[i].valid_) {
                                m_id = markers_pub.addVectorMarker(m_id, rand_pt, rand_pt + gradients[i].direction_*0.02, 0, 0, 1, 0.5, 0.005, "world");
                            }
                        }

                        if (r_map->getGradient(rand_pt, gr)) {
                            KDL::Vector prev = rand_pt;
                            rand_pt = rand_pt + gr * 0.02;
                            m_id = markers_pub.addVectorMarker(m_id, prev, rand_pt, 0, 1, 0, 0.5, 0.01, "world");
                        }
                        else {
                            std::cout << "gradient failed" << std::endl;
                            break;
                        }
                    }
                    path_length++;
                    if (path_length > 300) {
                        std::cout << "path length exceeded" << std::endl;
                        break;
                    }
                }
                markers_pub.addEraseMarkers(m_id, m_id+300);
            }

            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            m_id = 15000;
            m_id = addRobotModelVis(markers_pub, m_id, col_model, links_fk);

//            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);


            markers_pub.publish();
            ros::spinOnce();
            ros::Duration(0.02).sleep();
//            getchar();
        }
*/
//        double z = 1.2;
        double x = 0.7;
        while (ros::ok()) {
            int m_id = 0;
            if (x > 1.9) {
                break;
            }
//            z += 0.025;
            x += 0.01;
//            for (double x = lower_bound(0)+0.02; x < upper_bound(0); x += 0.01) {
//            for (double x = 0.7; x < 1.6; x += 0.02) {
            {
//                    double y = 0.0;
//                for (double y = lower_bound(1)+0.0125; y < upper_bound(1); y += 0.04) {
                for (double y = -0.5; y < 0.5; y += 0.01) {
//                    for (double z = lower_bound(2)+0.02; z < upper_bound(2); z += 0.01)
                    for (double z = 1.0; z < 2.1; z += 0.01)
                    {
//                        double z = 1.4;
                        KDL::Vector pt_W(x,y,z);
                        double val = 0.0;
                        {
                            val /= 1.1;
                            KDL::Vector gr;
                            if (r_map->getGradient(pt_W, gr)) {
                                double dist = 0.0;
                                r_map->getDistance(pt_W, dist);
                                dist = dist * 0.1;

                                double cr=1.0, cg=1.0, cb=1.0;
                                if (dist > 0) {
                                    cb -= std::min(1.0, dist);
                                    dist -= 1.0;
                                }
                                if (dist > 0) {
                                    cg -= std::min(1.0, dist);
                                    dist -= 1.0;
                                }
                                if (dist > 0) {
                                    cr -= std::min(1.0, dist);
                                    dist -= 1.0;
                                }

                                m_id = markers_pub.addSinglePointMarkerCube(m_id, pt_W+KDL::Vector(0.02, 0.02, 0.02), cr, cg, cb, 0.9, 0.0001, 0.01, 0.01, "world");
//                                m_id = markers_pub.addVectorMarker(m_id, pt_W, pt_W + gr*0.020, cr, cg, cb, 1, 0.006, "world");
//                                m_id = markers_pub.addSinglePointMarker(m_id, pt_W, 1, val, val, 0.5, 0.003, "world");
                            }
                            else {
                                m_id = markers_pub.addSinglePointMarker(m_id, pt_W, 0, 0, 1, 0, 0.003, "world");
                            }
                        }
                    }
                }
            }

            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            m_id = 15000;
            m_id = addRobotModelVis(markers_pub, m_id, col_model, links_fk);

//            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);


            markers_pub.publish();
            ros::spinOnce();
            ros::Duration(0.02).sleep();
//            getchar();
        }

        ros::Duration(3.0).sleep();
}

void generateBox(std::vector<KDL::Vector > &vertices, std::vector<int> &polygons, double size_x, double size_y, double size_z) {
        vertices.clear();
        polygons.clear();
        const int poly[] = {
        4, 0, 1, 2, 3,
        4, 4, 7, 6, 5,
        4, 0, 4, 5, 1,
        4, 3, 2, 6, 7,
        4, 5, 6, 2, 1,
        4, 0, 3, 7, 4,
        };
        const double vert[] = {
        0.5, -0.5, 0.5,
        0.5, 0.5, 0.5,
        -0.5, 0.5, 0.5,
        -0.5, -0.5, 0.5,
        0.5, -0.5, -0.5,
        0.5, 0.5, -0.5,
        -0.5, 0.5, -0.5,
        -0.5, -0.5, -0.5,
        };
        for (int i=0; i<8; i++) {
            vertices.push_back(KDL::Vector(vert[i*3]*size_x, vert[i*3+1]*size_y, vert[i*3+2]*size_z));
        }
        for (int i=0; i<6*5; i++) {
            polygons.push_back(poly[i]);
        }
}

void createEnvironment(self_collision::Link::VecPtrCollision &col_array, KDL::Frame &T_W_LOCK, KDL::Frame &T_W_BIN) {
        col_array.clear();

        // the walls
        std::vector<KDL::Vector > vertices;
        std::vector<int > polygons;

        KDL::Frame T_W_WALLS(KDL::Rotation::RotZ(0.0), KDL::Vector(0.4, 0, 0));
        KDL::Frame T_WALLS_DOOR(KDL::Vector(0.2, 0.92, 1.0));
        KDL::Frame T_DOOR_LOCK(KDL::Vector(-0.3, -0.05, 0.1));
        KDL::Frame T_DOOR_HANDLE(KDL::Vector(-0.25, -0.075, 0.2));
        T_W_LOCK = T_W_WALLS * T_WALLS_DOOR * T_DOOR_LOCK;
        T_W_BIN = KDL::Frame(KDL::Vector(0.23, -0.7, 0.5));
//        KDL::Frame T_W_C(KDL::Vector(1.0,0,1.5));
        KDL::Frame T_W_C(KDL::Vector(1.2,0,1.5));


        double wall_r = 1.0, wall_g = 1.0, wall_b = 1.0;
        generateBox(vertices, polygons, 0.2, 2.2, 2.2);
/*        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * KDL::Frame(KDL::Vector(-1.0, 0.0, 1.1)), "box 0.2 2.2 2.2") );
//        col_array.back()->geometry->setColor(1, 1, 1, 1);
        col_array.back()->geometry->setColor(wall_r+0.2, wall_g+0.2, wall_b+0.2, 1);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * KDL::Frame(KDL::Vector(1.1, 0.0, 1.1)), "box 0.2 2.2 2.2") );
        col_array.back()->geometry->setColor(wall_r+0.2, wall_g+0.2, wall_b+0.2, 1);

        generateBox(vertices, polygons, 2.0, 0.2, 2.2);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * KDL::Frame(KDL::Vector(0.0, 1.0, 1.1)), "box 2.0 0.2 2.2") );
        col_array.back()->geometry->setColor(wall_r, wall_g, wall_b+0.1, 1);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * KDL::Frame(KDL::Vector(0.0, -1.0, 1.1)), "box 2.0 0.2 2.2") );
        col_array.back()->geometry->setColor(wall_r+0.3, wall_g+0.3, wall_b+0.3, 1);
        generateBox(vertices, polygons, 2.4, 2.2, 0.2);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * KDL::Frame(KDL::Vector(0.0, 0.0, 2.3))) );
*/
        // the column
//        generateBox(vertices, polygons, 0.1, 0.1, 2.2);
//        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * KDL::Frame(KDL::Vector(0.2, -0.2, 1.1)), "box 0.1 0.1 2.2") );
//        col_array.back()->geometry->setColor(1.0, 0.0, 0.0, 1);

        // the door
/*
        generateBox(vertices, polygons, 0.8, 0.1, 2.0);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * T_WALLS_DOOR, "box 0.8 0.1 2.0") );
        col_array.back()->geometry->setColor(0.5, 0.2, 0, 1);
        // the lock
        col_array.push_back( self_collision::createCollisionSphere(0.02, T_W_LOCK) );
        col_array.back()->geometry->setColor(1, 1, 1, 1);
        // the handle
        generateBox(vertices, polygons, 0.15, 0.05, 0.02);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_WALLS * T_WALLS_DOOR * T_DOOR_HANDLE, "box 0.15 0.05 0.02") );
        col_array.back()->geometry->setColor(0.7, 0.7, 0.7, 1);
//*/
//*        // the bin
        generateBox(vertices, polygons, 0.02, 0.32, 0.4);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_BIN * KDL::Frame(KDL::Vector(0.16, 0.0, 0.2)), "box 0.02 0.32 0.4") );
        col_array.back()->geometry->setColor(0.5, 0.5, 0, 1);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_BIN * KDL::Frame(KDL::Vector(-0.16, 0.0, 0.2)), "box 0.02 0.32 0.4") );
        col_array.back()->geometry->setColor(0.5, 0.5, 0, 1);

        generateBox(vertices, polygons, 0.32, 0.02, 0.4);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_BIN * KDL::Frame(KDL::Vector(0.0, 0.16, 0.2)), "box 0.32 0.02 0.4") );
        col_array.back()->geometry->setColor(0.5, 0.5, 0, 1);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_BIN * KDL::Frame(KDL::Vector(0.0, -0.16, 0.2)), "box 0.32 0.02 0.4") );
        col_array.back()->geometry->setColor(0.5, 0.5, 0, 1);

        generateBox(vertices, polygons, 0.32, 0.32, 0.02);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_BIN * KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)), "box 0.32 0.32 0.02") );
        col_array.back()->geometry->setColor(0.5, 0.5, 0, 1);
//*/
        // the cabinet
//*
        double cabinet_a=1.0;
        generateBox(vertices, polygons, 0.4, 0.6, 0.02);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,-0.3)), "box 0.4 0.6 0.02") );
        col_array.back()->geometry->setColor(0, 0.5, 0.5, cabinet_a);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,0.0)), "box 0.4 0.6 0.02") );
        col_array.back()->geometry->setColor(0, 0.5, 0.5, cabinet_a);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,0.3)), "box 0.4 0.6 0.02") );
        col_array.back()->geometry->setColor(0, 0.5, 0.5, cabinet_a);

        generateBox(vertices, polygons, 0.02, 0.6, 0.6);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0.2,0,0)), "box 0.02 0.6 0.6") );
        col_array.back()->geometry->setColor(0, 0.5, 0.5, cabinet_a);

        generateBox(vertices, polygons, 0.4, 0.02, 0.6);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,-0.3,0)), "box 0.4 0.02 0.6") );
        col_array.back()->geometry->setColor(0, 0.5, 0.5, cabinet_a);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0.3,0)), "box 0.4 0.02 0.6") );
        col_array.back()->geometry->setColor(0, 0.5, 0.5, cabinet_a);
//*/
}

bool randomizedIkSolution(const boost::shared_ptr<KinematicModel> &kin_model, const KDL::Frame &T_W_G_dest, Eigen::VectorXd &q) {
    const int torso0_idx = kin_model->getJointIndex("torso_0_joint");
    const int free_idx = kin_model->getJointIndex("right_arm_2_joint");
    if (free_idx < 0) {
        std::cout << "ERROR: randomizedIkSolution: free_idx: " << free_idx << std::endl;
        return false;
    }
    IkReal eerot[9], eetrans[3], freeJoint;

    Eigen::VectorXd tmp_q(kin_model->getDofCount());

    int try_idx = 0;
    while (try_idx < 100) {
        ikfast::IkSolutionList<IkReal> solutions;

        // get random values for all joints
        for (int q_idx = 0; q_idx < kin_model->getDofCount(); q_idx++) {
            tmp_q(q_idx) = randomUniform(kin_model->getLowerLimit(q_idx), kin_model->getUpperLimit(q_idx));
        }

        KDL::Frame T_W_T2;
        kin_model->calculateFk(T_W_T2, "torso_link2", tmp_q);

        KDL::Frame T_T2_G_dest = T_W_T2.Inverse() * T_W_G_dest;

        eetrans[0] = T_T2_G_dest.p.x();
        eetrans[1] = T_T2_G_dest.p.y();
        eetrans[2] = T_T2_G_dest.p.z();

        eerot[0] = T_T2_G_dest.M(0,0);   eerot[1] = T_T2_G_dest.M(0,1);   eerot[2] = T_T2_G_dest.M(0,2);
        eerot[3] = T_T2_G_dest.M(1,0);   eerot[4] = T_T2_G_dest.M(1,1);   eerot[5] = T_T2_G_dest.M(1,2);
        eerot[6] = T_T2_G_dest.M(2,0);   eerot[7] = T_T2_G_dest.M(2,1);   eerot[8] = T_T2_G_dest.M(2,2);

        // random free joint value for arm
        freeJoint = tmp_q(free_idx);
        bool result = ComputeIk(eetrans, eerot, &freeJoint, solutions);

        if (result) {
            std::list<Eigen::VectorXd > valid_solutions;
            for (int sol_idx = 0; sol_idx < solutions.GetNumSolutions(); sol_idx++) {
                const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(sol_idx);
                std::vector<IkReal> solvalues(GetNumJoints());
                std::vector<IkReal> vsolfree(sol.GetFree().size());
                sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
                tmp_q( kin_model->getJointIndex("right_arm_0_joint") ) = solvalues[0];
                tmp_q( kin_model->getJointIndex("right_arm_1_joint") ) = solvalues[1];
                tmp_q( kin_model->getJointIndex("right_arm_2_joint") ) = solvalues[2];
                tmp_q( kin_model->getJointIndex("right_arm_3_joint") ) = solvalues[3];
                tmp_q( kin_model->getJointIndex("right_arm_4_joint") ) = solvalues[4];
                tmp_q( kin_model->getJointIndex("right_arm_5_joint") ) = solvalues[5];
                tmp_q( kin_model->getJointIndex("right_arm_6_joint") ) = solvalues[6];
/*
                // check
                IkReal joints[7];
                IkReal eetrans2[3], eerot2[9];
                joints[0] = tmp_q( kin_model->getJointIndex("right_arm_0_joint") );
                joints[1] = tmp_q( kin_model->getJointIndex("right_arm_1_joint") );
                joints[2] = tmp_q( kin_model->getJointIndex("right_arm_2_joint") );
                joints[3] = tmp_q( kin_model->getJointIndex("right_arm_3_joint") );
                joints[4] = tmp_q( kin_model->getJointIndex("right_arm_4_joint") );
                joints[5] = tmp_q( kin_model->getJointIndex("right_arm_5_joint") );
                joints[6] = tmp_q( kin_model->getJointIndex("right_arm_6_joint") );
                ComputeFk(joints, eetrans2, eerot2);
                double error = 0.0;
                for (int i = 0; i < 9; i++) {
                    error += fabs(eerot2[i]-eerot[i]);
                }
                for (int i = 0; i < 3; i++) {
                    error += fabs(eetrans2[i]-eetrans[i]);
                }

                KDL::Frame T_T2_G, T_W_G;
                kin_model->calculateFk(T_W_G, "right_HandGripLink", tmp_q);
                T_T2_G = T_W_T2.Inverse() * T_W_G;

                double error2 = 0.0;
                error2 = fabs(eerot2[0]-T_T2_G.M(0,0));
                error2 = fabs(eerot2[1]-T_T2_G.M(1,0));
                error2 = fabs(eerot2[2]-T_T2_G.M(2,0));
                error2 = fabs(eerot2[3]-T_T2_G.M(0,1));
                error2 = fabs(eerot2[4]-T_T2_G.M(1,1));
                error2 = fabs(eerot2[5]-T_T2_G.M(2,1));
                error2 = fabs(eerot2[6]-T_T2_G.M(0,2));
                error2 = fabs(eerot2[7]-T_T2_G.M(1,2));
                error2 = fabs(eerot2[8]-T_T2_G.M(2,2));
                error2 = fabs(eetrans2[0]-T_T2_G.p[0]);
                error2 = fabs(eetrans2[1]-T_T2_G.p[1]);
                error2 = fabs(eetrans2[2]-T_T2_G.p[2]);

                std::cout << "error: " << error << "   error2: " << error2 << std::endl;
*/

                // check bounds
                bool boundsOk = true;
                for (int q_idx = 0; q_idx < kin_model->getDofCount(); q_idx++) {
                    if (tmp_q(q_idx) <= kin_model->getLowerLimit(q_idx) || tmp_q(q_idx) >= kin_model->getUpperLimit(q_idx)) {
                        boundsOk = false;
                        break;
                    }
                }
                if (boundsOk) {
                    valid_solutions.push_back(tmp_q);
                }
            }
            if (!valid_solutions.empty()) {
                // get random valid solution
                std::list<Eigen::VectorXd >::const_iterator it = valid_solutions.begin();
                int sol_idx = rand() % valid_solutions.size();
                for (int i = 0; i < sol_idx; i++) {
                    it++;
                }
                q = (*it);
                return true;
            }
        }
        try_idx++;
    }

    return false;
}

KDL::Twist distanceMetric(const KDL::Frame &F_a_b1, const KDL::Frame &F_a_b2, const boost::shared_ptr<ReachabilityMap > &r_map) {
        KDL::Twist diff = KDL::diff(F_a_b1, F_a_b2, 1.0);
        if (diff.vel.Norm() < 0.05) {
            return diff;
        }
//        return diff;

        double dist = (F_a_b1.p - F_a_b2.p).Norm();
        KDL::Vector gr;
        if (r_map->getGradient(F_a_b1.p, gr)) {
            diff.vel = gr * dist;
//            int m_id = 6000;
//            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + gr*0.3, 0, 0, 1, 1, 0.005, "world");
//            markers_pub_.publish();
//            ros::spinOnce();
        }


        return diff;
}

void printJointLimits(const Eigen::VectorXd &q, const boost::shared_ptr<KinematicModel> &kin_model, const std::vector<std::string> &joint_names) {
                    int ndof = q.innerSize();
                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        double lo = kin_model->getLowerLimit(q_idx), up = kin_model->getUpperLimit(q_idx);
                        double f = (q(q_idx) - lo) / (up - lo);
                        int steps = 50;
                        int step = static_cast<int >(f*steps);
                        if (step >= steps) {
                            step = steps-1;
                        }
                        for (int s = 0; s < steps; s++) {
                            std::cout << ((s == step)?"*":".");
                        }
                        std::cout << "  ";

                        steps = 3;
                        step = static_cast<int >(f*steps);
                        if (step >= steps) {
                            step = steps-1;
                        }
                        for (int s = 0; s < steps; s++) {
                            std::cout << ((s == step)?"*":".");
                        }

                        std::cout << "    " << joint_names[q_idx] << std::endl;
                    }
}

bool checkCollision(const KDL::Vector &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model, double sphereRadius) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(sphereRadius, KDL::Frame(x));
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
}

visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.05;
  marker.scale.y = msg.scale * 0.05;
  marker.scale.z = msg.scale * 0.05;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DofMarker( interactive_markers::InteractiveMarkerServer &server, bool fixed, unsigned int interaction_mode, const KDL::Frame &T_W_M, bool show_6dof, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::Vector3 pos(T_W_M.p.x(), T_W_M.p.y(), T_W_M.p.z());
  double qx,qy,qz,qw;
  T_W_M.M.GetQuaternion(qx,qy,qz,qw);
  tf::Quaternion ori(qx,qy,qz,qw);
  tf::Transform pose(ori, pos);
  tf::poseTFToMsg(pose, int_marker.pose);
  int_marker.scale = 0.2;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server.insert(int_marker);

  server.setCallback(int_marker.name, feedback_cb );
//  server.setCallback(int_marker.name, boost::bind(&TestDynamicModel::processFeedback, this, _1) );
//  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//    menu_handler.apply( server, int_marker.name );
}

