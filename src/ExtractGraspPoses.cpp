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

#include <gpd_ros_wrapper/ExtractGraspPoses.h>


ExtractGraspPoses::ExtractGraspPoses (ros::NodeHandle* nh) {
    
    this->nh = nh;

//     nh->param<std::string>("camera_frame", camera_frame, "D435_head_camera_color_optical_frame");
    nh->param<std::string>("input_cloud_topic", input_cloud_topic, "/D435_head_camera/depth/color/point");
    nh->param<std::string>("selecting_frame", selecting_frame, "");
    nh->param<double>("still_selecting_frame_threshold", still_selecting_frame_threshold, 0.05);

    nh->param<int>("max_clusters", max_clusters, 5);
    nh->param<bool>("extend_grasps", extend_grasps, false);
    nh->param<bool>("publishSingleObjCloud", publishSingleObjCloud_, true);
    nh->param<bool>("publishSingleObjTF", publishSingleObjTF_, true);
    nh->param<bool>("publishSingleObjBoundingBox", publishSingleObjBoundingBox_, true);
    nh->param<bool>("publishGraspsTf", publishGraspsTf_, true);
    if (!nh->getParam("gpdConfig", gpdConfig)) {
        throw std::invalid_argument("Rosparam 'gpdConfig' not found on parameter server!");
    }

    nh->param<bool>("tf_listener", tf_listen, false);

    if (tf_listen) {
        tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);
    }
    
    ROS_INFO_STREAM("Waiting for point cloud on topic: " << input_cloud_topic);
    auto msg = ros::topic::waitForMessage<PointCloud>(input_cloud_topic, *nh);
    if (msg->header.frame_id.empty()) {
        throw std::runtime_error("Error: topic '" + input_cloud_topic + "' provide point clouds with empty frame_id");
    }
    cloud_ref_frame = msg->header.frame_id;
    ROS_INFO_STREAM("Message on " << input_cloud_topic << " arrived");
    cloud_sub = nh->subscribe<PointCloud>(input_cloud_topic, 1, &ExtractGraspPoses::cloudClbk, this);
    
    if (publishSingleObjCloud_) {
        cloud_tmp_pub.resize(max_clusters);
        for (int i=0; i<max_clusters; i++) {
            cloud_tmp_pub.at(i) = nh->advertise<PointCloud>("cloud_tmp_"+ std::to_string(i), 1);
        }
    }
    
    object_clusters.resize(max_clusters); //max cluster we detect
    for (int i=0; i<object_clusters.size(); i++) {
        object_clusters.at(i)._cloud = boost::make_shared<PointCloud>();
    }
    
    cloud = boost::make_shared<PointCloud>();
    
    if (publishSingleObjBoundingBox_) {
        marker_pub = nh->advertise<visualization_msgs::MarkerArray>("objects_bounding", 1);
    }
    
    selected_object_srv = nh->advertiseService("object_selected", &ExtractGraspPoses::selectedObjectClbk, this);
    
    //for some debug
    //tmp_pub = nh->advertise<sensor_msgs::Image>("/image_trial",1);

    //check if file exist && the path to model is correct
    config_file_ = std::make_unique<gpd::util::ConfigFile>(gpdConfig);
    if (!config_file_->ExtractKeys()) {
        throw std::invalid_argument("Error reading config file " + gpdConfig);
    }
    std::string weight_file = config_file_->getValueOfKeyAsString("weights_file", "");
    if (weight_file.empty()) {
        throw std::invalid_argument("weights_file param not present in the config file " + gpdConfig);
    }
    if (!std::filesystem::exists(weight_file)) {
        throw std::invalid_argument("Not found weights file " + weight_file);
    }

    bool a = config_file_->keyExists("centered_at_origin");

    grasp_detector = std::make_unique<gpd::GraspDetector>(gpdConfig);
    clod_xyzrgba_ = boost::make_shared<PointCloudXYZRGBA>();
    gpd_cloud_ = std::make_unique<gpd::util::Cloud>();

    //to exract some more params necessary outside, like "centered_at_origin" and view points ("camera_position")
    // Set the camera position. Assumes a single camera view.
    std::vector<double> camera_position =
        config_file_->getValueOfKeyAsStdVectorDouble("camera_position",
                                                    "0.0 0.0 0.0");
    camera_view_point << camera_position[0], camera_position[1], camera_position[2];
 
}

int ExtractGraspPoses::run () {

    if (!getTransforms()) {
        return -1;
    }

    //change reference frame
    //pcl_ros::transformPointCloud (cloud_ref_frame, *cloud, *cloud, tf_buffer);

    if (! clusterExtraction()) {
        return -1;
    }

    momentOfInertia();
    
    findPoints(
        refcloud_T_selecting_frame.transform.translation.x, 
        refcloud_T_selecting_frame.transform.translation.y,
        refcloud_T_selecting_frame.transform.translation.z);

    //if selecting_frame moved, always check, if not, check only if grasps are empty 
    if (selecting_frame_moved || grasps.size() == 0)  { 
        graspDetection();
        finalizeGrasps(0,0,0, extend_grasps);
    }

    publishStuff();

    return 0;
}

int ExtractGraspPoses::run_no_tf_input() {

    //change reference frame
    //pcl_ros::transformPointCloud (cloud_ref_frame, *cloud, *cloud, tf_buffer);

    if (! clusterExtraction()) {
        return -1;
    }

    momentOfInertia();
    
    //select cluster "manually" instead of finding point from tf
    //TODO: redo if selecting another cluster?
    if (selectCluster()) {
        if (!graspDetection()) {
            ROS_ERROR_STREAM("Grasp detection failed");
        } 
        if (grasps.size() == 0) {
            ROS_WARN_STREAM("No grasps detected");
        }
        if (!finalizeGrasps(0,0,0, extend_grasps)) {
            ROS_ERROR_STREAM("Finalizing grasps failed");
        }
    }

    publishStuff();

    return 0;

}

bool ExtractGraspPoses::clusterExtraction(){

    if (cloud->size() == 0) {
        ROS_WARN("Point cloud from '%s' empty", input_cloud_topic.c_str());
        return false;
    }

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    // pcl::VoxelGrid<Point> vg;
    // vg.setInputCloud (cloud);
    // vg.setLeafSize (0.01f, 0.01f, 0.01f);
    // vg.filter (*cloud);
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (70);
    ec.setMaxClusterSize (2500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    if (object_clusters.size() < cluster_indices.size()) {
        ROS_WARN_STREAM( "WARN! clusters found exceed the max number of " <<
            object_clusters.size() << " I will not store the exceedent ones ");
    }
    
    n_clusters = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

        object_clusters.at(n_clusters)._cloud->points.clear();
        object_clusters.at(n_clusters)._cloud->header.frame_id = cloud->header.frame_id;
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            object_clusters.at(n_clusters)._cloud->push_back ((*cloud)[*pit]); //*
        }
        object_clusters.at(n_clusters)._cloud->width = object_clusters.at(n_clusters)._cloud->size ();
        object_clusters.at(n_clusters)._cloud->height = 1;
        object_clusters.at(n_clusters)._cloud->is_dense = true;
        
        n_clusters++;
        
        if (n_clusters >= object_clusters.size()) {
            
            break;
        }
    }

    //std::cout << "Found " << cluster_indices.size() << " clusters " << std::endl;
    return true;
}

bool ExtractGraspPoses::momentOfInertia(const std::string &mode) {

    bool ret = true;

    if (mode.compare("aabb") == 0) {
        for (int i=0; i<n_clusters; i++) {
            
            if (!object_clusters.at(i).momentOfInertiaAABB()) {
                //return always false IDK why...
                //std::cout << "WARN: momentOfInertiaAABB gets return false for cluster '" << std::to_string(i) << "'" << std::endl;
            }
        }  
    } else if (mode.compare("obb") == 0) {
        for (int i=0; i<n_clusters; i++) {
            
            if (!object_clusters.at(i).momentOfInertiaOBB()) {
               ROS_WARN_STREAM ("momentOfInertiaOBB gets return false for cluster '" << std::to_string(i) << "'");
                ret = false;
            }
        }

    } else {
        ROS_ERROR_STREAM("momentOfInertia " << mode << " not known");
        ret = false;
    }

    return ret;
}

const std::vector<ObjectCluster>* ExtractGraspPoses::getObjectClusters() const {

    return &object_clusters;
}

const ObjectCluster* ExtractGraspPoses::getSelectedObjectCluster() const {

    if (selected_cluster < 0) {
        ROS_WARN_STREAM ("No cluster selected!");
        return nullptr;
    }

    return &(object_clusters.at(selected_cluster));
}

bool ExtractGraspPoses::selectCluster() {

    selected_cluster = -1;
    if (object_clusters.size() > 0) {
        selected_cluster = 0; //TODO
        object_clusters.at(selected_cluster).selectCluster();
    } else {
        ROS_WARN("No clusters found, cannot select one");
        return false;
    }
    return true;
}

bool ExtractGraspPoses::findPoints(double x, double y, double z) {

    selected_cluster = -1;
    Point searchPoint(x, y, z);
    for (int i=0; i<n_clusters; i++) {
        
        if (object_clusters.at(i).findPoint(searchPoint)) {
            selected_cluster = i;
            break;
        } 
    }

    return (selected_cluster != -1);
}

bool ExtractGraspPoses::graspDetection(
    bool filter_direction, const std::vector<std::array<double,3>>& directions, const double& thresh_rad, 
    bool filter_position, const std::array<double,3>& position_upper, const std::array<double,3>& position_lower) {

    if (selected_cluster < 0) {
        return false;
    }

    //convert cloud
    // TODO: set alpha channel to 1
    // GPD required XYZRGBA
    pcl::copyPointCloud(*object_clusters.at(selected_cluster)._cloud.get(), *clod_xyzrgba_.get());

    // Construct the cloud camera
    gpd_cloud_.reset(new gpd::util::Cloud(clod_xyzrgba_, 0, camera_view_point, false));

    // Preprocess the point cloud.
    grasp_detector->preprocessPointCloud(*gpd_cloud_);

    // If the object is centered at the origin, reverse all surface normals.
    bool centered_at_origin =
        config_file_->getValueOfKey<bool>("centered_at_origin", false);
    if (centered_at_origin) {
        //printf("Reversing normal directions ...\n");
        gpd_cloud_->setNormals(gpd_cloud_->getNormals() * (-1.0));
    }

    grasps = grasp_detector->detectGrasps(*gpd_cloud_, 
        filter_direction, directions, thresh_rad, 
        filter_position, position_upper, position_lower);

    return true;
}

bool ExtractGraspPoses::graspDetection(
    bool filter_direction, const std::array<double,3>& thresh_magnitude_normalized_upper, const std::array<double,3>& thresh_magnitude_normalized_lower,
    bool filter_position, const std::array<double,3>& position_upper, const std::array<double,3>& position_lower) {

    if (selected_cluster < 0) {
        return false;
    }

    //convert cloud
    // TODO: set alpha channel to 1
    // GPD required XYZRGBA
    pcl::copyPointCloud(*object_clusters.at(selected_cluster)._cloud.get(), *clod_xyzrgba_.get());

    // Construct the cloud camera
    gpd_cloud_.reset(new gpd::util::Cloud(clod_xyzrgba_, 0, camera_view_point, false));

    // Preprocess the point cloud.
    grasp_detector->preprocessPointCloud(*gpd_cloud_);

    // If the object is centered at the origin, reverse all surface normals.
    bool centered_at_origin =
        config_file_->getValueOfKey<bool>("centered_at_origin", false);
    if (centered_at_origin) {
        //printf("Reversing normal directions ...\n");
        gpd_cloud_->setNormals(gpd_cloud_->getNormals() * (-1.0));
    }

    grasps = grasp_detector->detectGrasps(*gpd_cloud_, 
        filter_direction, thresh_magnitude_normalized_upper, thresh_magnitude_normalized_lower, 
        filter_position, position_upper, position_lower);

    return true;
}

bool ExtractGraspPoses::finalizeGrasps(const double& offset_x, const double& offset_y, const double& offset_z, bool extend_grasps) {
 
    grasps_poses.resize(grasps.size());
    uint i = 0;
    for (const auto &grasp : grasps) {

        //gpd has x axis going out the ee and the z as normal
        const Eigen::Affine3d gdp_T_eeGoal = 
            Eigen::Translation3d(0,0,0) * Eigen::Quaterniond(0.7071068, 0, 0.7071068, 0); //w, x, y, z

        //a bit of offes to not go against the object
        const Eigen::Affine3d eeGoal_T_eeGoalFinal =  
            Eigen::Translation3d(offset_x,offset_y,offset_z) * Eigen::Quaterniond(1,0,0,0); //w, x, y, z

        //std::cout << "OFFEST:  " << gdp_T_eeGoal.translation().transpose() << std::endl;
        // std::cout << gdp_T_eeGoal.linear() << std::endl;

        const Eigen::Affine3d cloudRef_T_gdp = 
            Eigen::Translation3d(grasp->getPosition()) * Eigen::Quaterniond(grasp->getOrientation());

        const Eigen::Affine3d cloudRef_T_eeGoalFinal = cloudRef_T_gdp * gdp_T_eeGoal * eeGoal_T_eeGoalFinal;

        geometry_msgs::Pose grasp_pose;
        tf::poseEigenToMsg(cloudRef_T_eeGoalFinal, grasp_pose);

        grasps_poses.at(i) = grasp_pose;
        i++;
    }

    if (extend_grasps) {
        grasps_poses_extended.resize(grasps.size()*2);
        uint i = 0;
        for (const auto &grasp : grasps_poses) { 
            grasps_poses_extended.at(i) = grasp;
            grasps_poses_extended.at(i+1) = grasp;
            //rotate quaternion about 180 deg in z
            Eigen::Quaterniond q_180_z, q;
            q_180_z.x() = 0;
            q_180_z.y() = 0;
            q_180_z.z() = 1;
            q_180_z.w() = 0;
            tf::quaternionMsgToEigen(grasps_poses_extended.at(i+1).orientation, q);
            q = q * q_180_z;
            q.normalize();
            tf::quaternionEigenToMsg(q, grasps_poses_extended.at(i+1).orientation);
         
            i+=2;  
        }
    }

    return true;
}

std::vector<geometry_msgs::Pose> ExtractGraspPoses::getGraspsPoses() const {
    return grasps_poses;
}
std::vector<geometry_msgs::Pose> ExtractGraspPoses::getExtendedGraspsPoses() const {
    return grasps_poses_extended;
}

geometry_msgs::TransformStamped ExtractGraspPoses::getRefCloudTSelectingFrame() const {
    return refcloud_T_selecting_frame;
}

void ExtractGraspPoses::cloudClbk(const PointCloud::ConstPtr& msg)
{
    *cloud = *msg;
}

bool ExtractGraspPoses::selectedObjectClbk(gpd_ros_wrapper::ObjectClusterSrv::Request &req, 
                                                gpd_ros_wrapper::ObjectClusterSrv::Response &res) {
    
    //req is empty
    
    res.header.stamp = ros::Time::now();
    
    if (selected_cluster == -1) {
        ROS_WARN_STREAM_THROTTLE(1, "No clusters selected!");
        
    } else {
    
        //cloud frame id is cloud_ref_frame
        res.header.frame_id = object_clusters.at(selected_cluster)._cloud->header.frame_id;
        res.child_frame_id = "box_cloud_" + std::to_string(selected_cluster);
        
        res.ref_T_object.translation.x = object_clusters.at(selected_cluster)._position (0); 
        res.ref_T_object.translation.y = object_clusters.at(selected_cluster)._position (1);
        res.ref_T_object.translation.z = object_clusters.at(selected_cluster)._position (2);
        
        res.ref_T_object.rotation.x = object_clusters.at(selected_cluster)._rotation.x();
        res.ref_T_object.rotation.y = object_clusters.at(selected_cluster)._rotation.y();
        res.ref_T_object.rotation.z = object_clusters.at(selected_cluster)._rotation.z();
        res.ref_T_object.rotation.w = object_clusters.at(selected_cluster)._rotation.w();
        
        res.dimensions.x = object_clusters.at(selected_cluster)._dimensions(0);
        res.dimensions.y = object_clusters.at(selected_cluster)._dimensions(1);
        res.dimensions.z = object_clusters.at(selected_cluster)._dimensions(2);
        
    }
        
    
    return true;
}

bool ExtractGraspPoses::publishStuff() {

    bool ret = true;
    ret = ret && publishClusters(publishSingleObjCloud_, publishSingleObjTF_, publishSingleObjBoundingBox_);
    if (publishGraspsTf_) {
        ret = ret && publishGrasps();
    }
    
    return ret;
}

bool ExtractGraspPoses::publishClusters(bool publishSingleObjCloud, bool publishSingleObjBoundingBox, bool publishSingleObjTF) {

    if (publishSingleObjCloud) {
        for (int i=0; i<n_clusters; i++) {
            cloud_tmp_pub.at(i).publish(object_clusters.at(i)._cloud);
        }
    }
    
    if (publishSingleObjBoundingBox)
    {   
        markerArrayMsg.markers.resize(n_clusters);

        for (int i=0; i<n_clusters; i++) {
            object_clusters.at(i).fillMarker(i);
            markerArrayMsg.markers.at(i) = object_clusters.at(i)._marker;
        }
        
        marker_pub.publish(markerArrayMsg);
    }
    
    if (publishSingleObjTF) { 

        transforms.resize(n_clusters);

        for (int i=0; i<n_clusters; i++) {
            object_clusters.at(i).fillTransform(i);
            transforms.at(i) = object_clusters.at(i)._ref_T_cloud;
        }

        tf_broadcaster.sendTransform(transforms);
    }
    return true;
}

bool ExtractGraspPoses::publishGrasps(unsigned int publishGraspsTfMaxTf) {

    if (grasps.size() == 0) {
        ROS_WARN_STREAM ("Publishing grasps error: no grasps to pub");
        return false;
    }

    int i = 0;

    const std::vector<geometry_msgs::Pose>* grasps_poses_alias = nullptr;
    if (extend_grasps) {
        grasps_poses_alias = &grasps_poses_extended;
    } else {
        grasps_poses_alias = &grasps_poses;
    }
    
    for (const auto& grasp_pose : *grasps_poses_alias)
    {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = cloud_ref_frame; // grasp are wrt cloud frame
        tf.child_frame_id = "grasp_detected_" + std::to_string(i);
        tf.transform.translation.x = grasp_pose.position.x;
        tf.transform.translation.y = grasp_pose.position.y;
        tf.transform.translation.z = grasp_pose.position.z;

        tf.transform.rotation.x = grasp_pose.orientation.x;
        tf.transform.rotation.y = grasp_pose.orientation.y;
        tf.transform.rotation.z = grasp_pose.orientation.z;
        tf.transform.rotation.w = grasp_pose.orientation.w;

        grasp_tf.sendTransform(tf);
        i++;
        if (i>=publishGraspsTfMaxTf) {
            break;
        }
    }
    return true;
}


bool ExtractGraspPoses::getTransforms()
{
    if (!tf_listen) {

        ROS_ERROR("Called getTransforms but option in the costructor for listening to them was false");
        return false;
    }

    try {
        
        refcloud_T_selecting_frame = tf_buffer.lookupTransform(cloud_ref_frame, selecting_frame, ros::Time(0));
      
    } 
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    if (std::sqrt (
            std::pow(refcloud_T_selecting_frame.transform.translation.x - refcloud_T_selecting_frame_prev.transform.translation.x, 2) +
            std::pow(refcloud_T_selecting_frame.transform.translation.y - refcloud_T_selecting_frame_prev.transform.translation.y, 2) 
            + std::pow(refcloud_T_selecting_frame.transform.translation.z - refcloud_T_selecting_frame_prev.transform.translation.z, 2) 
            ) < still_selecting_frame_threshold) {
        
        selecting_frame_moved = false;
        
    } else { 
        selecting_frame_moved = true;
    }
    refcloud_T_selecting_frame_prev = refcloud_T_selecting_frame;

    return true;

}
