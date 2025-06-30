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


#ifndef OBJECT_CLUSTER_H
#define OBJECT_CLUSTER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGBA PointXYZRGBA;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<PointXYZRGBA> PointCloudXYZRGBA;

class ObjectCluster {
    
public:
    
    ObjectCluster();
        
    PointCloud::Ptr _cloud;
    
    bool momentOfInertiaOBB();
    bool momentOfInertiaAABB();

    void fillMarker(unsigned id);
    moveit_msgs::CollisionObject getCollisionObject(unsigned id) const;
    void fillTransform(unsigned id);
    geometry_msgs::PoseStamped getPose() const;
    std::array<double,3> getDimensions() const;
        
    visualization_msgs::Marker _marker;
    geometry_msgs::TransformStamped _ref_T_cloud;
    
    bool findPoint(Point searchPoint, float radius = 0.03);
    bool selectCluster(bool select = true);
        
    pcl::MomentOfInertiaEstimation <Point> _feature_extractor;
    
    //This will be different according to AABB or OBB method
    Eigen::Vector3f _dimensions;
    Eigen::Vector3f _position;
    Eigen::Quaternionf _rotation;

    
private:
    
    pcl::KdTreeFLANN<Point> _kdtree;
    std::vector<int> _point_idx_found; //to store index of surrounding points 
    std::vector<float> _point_radius_squared_distance; // to store distance to surrounding points
    bool _selected_cluster;


};

#endif // OBJECT_CLUSTER_H