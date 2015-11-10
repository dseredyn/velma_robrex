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
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "kdl_conversions/kdl_msg.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

//#include "barrett_hand_controller_msgs/Empty.h"
//#include <barrett_hand_controller_msgs/BHMoveAction.h>

//#include "velma_dyn_model.h"
#include <collision_convex_model/collision_convex_model.h>
//#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
//#include "planer_utils/task_col.h"
//#include "planer_utils/task_hand.h"
//#include "planer_utils/task_jlc.h"
//#include "planer_utils/task_wcc.h"
//#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"
//#include "planer_utils/simulator.h"
//#include "planer_utils/reachability_map.h"

//#include "experiments_utilities.h"
//#include "ompl_utilities.h"
//#include "velma_robrex_utilities.h"
//#include "barrett_hand_interface.h"
//#include "velma_interface.h"

class PCFilter {
    const double PI;
    ros::NodeHandle nh_;
    MarkerPublisher markers_pub_;
    bool point_cloud_processed_;
    double tolerance_;

    tf::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
    tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
    ros::Publisher pub_pc_;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloud;
    PclPointCloud pc_;
    ros::Time pc_stamp_;
    std::string pc_frame_id_;

public:

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
        if (point_cloud_processed_) {
            point_cloud_processed_ = false;
            pcl::fromROSMsg(*cloud, pc_);
            pc_stamp_ = cloud->header.stamp;
            pc_frame_id_ = cloud->header.frame_id;
        }
    }

    PCFilter() :
        PI(3.141592653589793),
        nh_(),
        markers_pub_(nh_),
        point_cloud_processed_(true),
        tolerance_(0.04)
    {
        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, tf_listener_, "world", 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&PCFilter::insertCloudCallback, this, _1));

        pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2 >("cloud_out", 10);
    }

    ~PCFilter() {
    }

    int getchar2() {
        int c = getchar();
        int c2 = getchar();
        if (c2 != '\n') {
            return c2;
        }
        return c;
    }

    void spin() {

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

        // set colors for each link
//        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
//            for (self_collision::Link::VecPtrCollision::iterator it = col_model->getLink(l_idx)->collision_array.begin(); it != col_model->getLink(l_idx)->collision_array.end(); it++) {
//                (*it)->geometry->setColor(0,1,0,0.5);
//            }
//        }

        boost::shared_ptr< self_collision::Collision > shpere = self_collision::createCollisionSphere(tolerance_, KDL::Frame());

        ros::Rate loop_rate(5);
        int errors = 0;
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();

            if (errors > 5) {
                point_cloud_processed_ = true;
            }

            if (!point_cloud_processed_) {

                tf::StampedTransform tf_W_C;
                try {
                    tf_listener_.lookupTransform("world", pc_frame_id_, pc_stamp_, tf_W_C);
                } catch(tf::TransformException& ex){
                    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                    errors++;
                    continue;
                }
                geometry_msgs::TransformStamped tfm_W_C;
                KDL::Frame T_W_C;
                tf::transformStampedTFToMsg(tf_W_C, tfm_W_C);
                tf::transformMsgToKDL(tfm_W_C.transform, T_W_C);

                std::vector<KDL::Frame > links_tf(col_model->getLinksCount());
                std::set<std::string > colliding_links;
                std::vector<bool > pt_col(pc_.points.size(), false);
                bool tf_error = false;
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    const boost::shared_ptr< self_collision::Link > plink = col_model->getLink(l_idx);
                    if (plink->collision_array.size() == 0) {
                        continue;
                    }
                    tf::StampedTransform tf_W_L;
                    try {
                        tf_listener_.lookupTransform("world", plink->name, pc_stamp_, tf_W_L);
                    } catch(tf::TransformException& ex){
                        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                        tf_error = true;
                        break;
                    }
                    geometry_msgs::TransformStamped tfm_W_L;
                    KDL::Frame T_W_L;
                    tf::transformStampedTFToMsg(tf_W_L, tfm_W_L);
                    tf::transformMsgToKDL(tfm_W_L.transform, T_W_L);
                    KDL::Frame T_C_L = T_W_C.Inverse() * T_W_L;
                    links_tf[l_idx] = T_W_L;

                    for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                        if (pt_col[pidx]) {
                            continue;
                        }
                        KDL::Frame T_C_P(KDL::Vector(pc_.points[pidx].x, pc_.points[pidx].y, pc_.points[pidx].z));
                        if (self_collision::checkCollision(shpere, T_C_P, plink, T_C_L)) {
                            pt_col[pidx] = true;
                            colliding_links.insert( plink->name );
                        }
                    }
                }
                if (tf_error) {
                    errors++;
                    continue;
                }

                PclPointCloud pc_out;
                for (int pidx = 0; pidx < pc_.points.size(); pidx++) {
                    if (pt_col[pidx]) {
//                        pcl::PointXYZRGB pt = pc_.points[pidx];
//                        pt.r = 0;
//                        pt.g = 0;
//                        pt.b = 1;
//                        pc_out.push_back( pt );
                        continue;
                    }
                    pc_out.push_back( pc_.points[pidx] );
                }

                sensor_msgs::PointCloud2 ros_pc_out;
                toROSMsg(pc_out, ros_pc_out);
                ros_pc_out.header.stamp = pc_stamp_;
                ros_pc_out.header.frame_id = pc_frame_id_;

//                std::cout << pc_frame_id_ << "  " << errors << std::endl;
//                for (std::set<std::string >::const_iterator it = colliding_links.begin(); it != colliding_links.end(); it++) {
//                    std::cout << (*it) << "  ";
//                }
//                std::cout << std::endl;
                pub_pc_.publish(ros_pc_out);
                point_cloud_processed_ = true;
                errors = 0;

//                addRobotModelVis(markers_pub_, 0, col_model, links_tf);
//                markers_pub_.publish();
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_filter");
    PCFilter pcf;
    pcf.spin();
    return 0;
}


