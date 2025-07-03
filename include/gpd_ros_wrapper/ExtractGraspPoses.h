/*
 * Copyright 2025 <Davide Torielli> <toridebraus@gmail.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef EXTRACT_GRASP_POSES_H
#define EXTRACT_GRASP_POSES_H

#include <filesystem>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/topic.h>

#include <gpd_ros_wrapper/ObjectCluster.h>
#include <gpd_ros_wrapper/PosesStamped.h>
#include <gpd_ros_wrapper/ObjectClusterSrv.h>
#include <gpd_ros_wrapper/SelectObjectSrv.h>
#include <gpd_ros_wrapper/FilterGraspDirectionSrv.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/fpfh_omp.h>

#include <gpd/grasp_detector.h>

class ExtractGraspPoses {
    
public:
    ExtractGraspPoses(ros::NodeHandle* nh);

    std::string cloud_ref_frame;
    std::string selecting_frame;
        
    bool clusterExtraction();
    bool momentOfInertia(const std::string &mode = "aabb");
    const std::vector<ObjectCluster>* getObjectClusters() const;
    const ObjectCluster* getSelectedObjectCluster() const;
    bool findPoints(double x, double y, double z);
    bool graspDetection(
        bool filter_direction, const std::vector<std::array<double,3>>& directions, const double& thresh_rad,
        bool filter_position, const std::array<double,3>& position_upper, const std::array<double,3>& position_lower);

    bool graspDetection2(
        bool filter_direction, const std::array<double,3>& thresh_magnitude_normalized_upper, const std::array<double,3>& thresh_magnitude_normalized_lower,
        bool filter_position, const std::array<double,3>& position_upper, const std::array<double,3>& position_lower);
    bool finalizeGrasps(const double& offset_x = 0, const double& offset_y=0, const double& offset_z=0, bool extended=false);
    bool publishStuff();
    bool publishClusters(bool publishSingleObjCloud, bool publishSingleObjBoundingBox, bool publishSingleObjTF);
    bool publishGraspsTf(unsigned int publishGraspsTfMaxTf = 5);

    geometry_msgs::TransformStamped getRefCloudTSelectingFrame() const;
    std::vector<geometry_msgs::Pose> getGraspsPoses() const;
    std::vector<geometry_msgs::Pose> getExtendedGraspsPoses() const;

    int run();
    int run_no_tf_input();
    
private:
    ros::NodeHandle* nh;
    
//     std::string camera_frame;
    std::string input_cloud_topic;
    
    bool tf_listen;
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::TransformStamped refcloud_T_selecting_frame;
    geometry_msgs::TransformStamped refcloud_T_selecting_frame_prev;
    double still_selecting_frame_threshold;
    double selecting_frame_moved = false;
    bool getTransforms();

    ros::Subscriber cloud_sub;
    void cloudClbk(const PointCloud::ConstPtr& msg);
    ros::Publisher cloud_plane_pub, cloud_objects_pub;
    std::vector<ros::Publisher> cloud_tmp_pub;
    
    //bounding box marker pub
    ros::Publisher marker_pub;
    visualization_msgs::MarkerArray markerArrayMsg;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::vector<geometry_msgs::TransformStamped> transforms;

    ros::Publisher grasp_poses_pub;
    gpd_ros_wrapper::PosesStamped grasp_poses_msg;
    void publishGraspPoses();
    
    ros::ServiceServer get_selected_object_srv;
    bool getSelectedObjectClbk(gpd_ros_wrapper::ObjectClusterSrv::Request &req, gpd_ros_wrapper::ObjectClusterSrv::Response &res);

    ros::ServiceServer select_object_srv;
    bool selectObjectClbk(gpd_ros_wrapper::SelectObjectSrv::Request &req, gpd_ros_wrapper::SelectObjectSrv::Response &res);

    ros::ServiceServer filter_grasp_direction_srv;
    bool filterGraspDirectionClbk(gpd_ros_wrapper::FilterGraspDirectionSrv::Request &req, gpd_ros_wrapper::FilterGraspDirectionSrv::Response &res);
    
    //phases methods
    std::vector<ObjectCluster> object_clusters;
    unsigned int n_clusters = 0;
    int max_clusters;
    bool extend_grasps;
    bool publishSingleObjCloud_;
    bool publishSingleObjTF_;
    bool publishSingleObjBoundingBox_;
    bool publishGraspsTf_;
    PointCloud::Ptr cloud;  
        
    //ros::Publisher tmp_pub;
    
    int selected_cluster = -1;
    bool _filter_direction;
    std::vector<std::array<double,3>> _grasping_directions;
    double _thresh_direction_rad;
    bool _filter_position;
    std::array<double,3> _grasping_position_upper;
    std::array<double,3> _grasping_position_lower;

    //Grasp with gpd
    std::string gpdConfig;
    std::unique_ptr<gpd::GraspDetector> grasp_detector;
    std::unique_ptr<gpd::util::ConfigFile> config_file_;
    PointCloudXYZRGBA::Ptr clod_xyzrgba_;
    std::unique_ptr<gpd::util::Cloud> gpd_cloud_;
    tf2_ros::TransformBroadcaster grasp_tf;
    Eigen::Vector3d camera_view_point;
    //TODO grasps should be part of the ObjectCluster?
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;
    std::vector<geometry_msgs::Pose> grasps_poses;
    std::vector<geometry_msgs::Pose> grasps_poses_extended;

};



#endif // EXTRACT_GRASP_POSES_H
