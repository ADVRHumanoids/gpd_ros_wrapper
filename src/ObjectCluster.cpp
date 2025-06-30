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

 #include <gpd_ros_wrapper/ObjectCluster.h> 

 ObjectCluster::ObjectCluster() {
    
    _marker.ns = "object_cluster_bboxes";
    _marker.type = visualization_msgs::Marker::CUBE;
    _marker.action = visualization_msgs::Marker::ADD;
    _marker.color.a = 0.45; // Don't forget to set the alpha!
    _marker.color.r = 0.5;
    _marker.color.g = 1.0;
    _marker.color.b = 1.0;
    _marker.pose.orientation.x = 0.0;
    _marker.pose.orientation.y = 0.0;
    _marker.pose.orientation.z = 0.0;
    _marker.pose.orientation.w = 1.0;

    _rotation.w() = 1.0; //avoid annoying warning
    
    _kdtree.setSortedResults (true);
    _selected_cluster = false;
    
}

bool ObjectCluster::momentOfInertiaOBB() {
    
    _feature_extractor.setInputCloud (_cloud);
    _feature_extractor.compute ();

    bool ret_value = true;

    pcl::PointXYZ _min_point_OBB;
    pcl::PointXYZ _max_point_OBB;
    pcl::PointXYZ _position_OBB;    
    Eigen::Matrix3f _rotational_matrix_OBB;
    
    ret_value = (ret_value && _feature_extractor.getOBB (_min_point_OBB, _max_point_OBB, _position_OBB, _rotational_matrix_OBB));
    ret_value = (ret_value && _feature_extractor.getMassCenter (_position));
        
    _rotation = Eigen::Quaternionf (_rotational_matrix_OBB);
    _rotation.normalize();
    
    _dimensions << 
        std::abs(_max_point_OBB.x - _min_point_OBB.x),
        std::abs(_max_point_OBB.y - _min_point_OBB.y),
        std::abs(_max_point_OBB.z - _min_point_OBB.z);
        
        
    return ret_value;
}

bool ObjectCluster::momentOfInertiaAABB() {
    
    _feature_extractor.setInputCloud (_cloud);
    _feature_extractor.compute ();

    pcl::PointXYZ _min_point_AABB;
    pcl::PointXYZ _max_point_AABB;
    bool ret_value = true;
    ret_value = (ret_value && _feature_extractor.getAABB (_min_point_AABB, _max_point_AABB));
    
    _dimensions <<
        std::abs(_max_point_AABB.x - _min_point_AABB.x),
        std::abs(_max_point_AABB.y - _min_point_AABB.y),
        std::abs(_max_point_AABB.z - _min_point_AABB.z);

    _rotation.setIdentity();
    
    _position <<
        0.5 * (_max_point_AABB.x + _min_point_AABB.x),
        0.5 * (_max_point_AABB.y + _min_point_AABB.y),
        0.5 * (_max_point_AABB.z + _min_point_AABB.z);
    
    return ret_value;
}

void ObjectCluster::fillMarker(unsigned id) {
    
    _marker.id = id;
    _marker.header.frame_id = _cloud->header.frame_id;
    _marker.header.stamp = ros::Time();
    _marker.pose.position.x = _position (0); 
    _marker.pose.position.y = _position (1); 
    _marker.pose.position.z = _position (2); 
    _marker.pose.orientation.x = _rotation.x();
    _marker.pose.orientation.y = _rotation.y();
    _marker.pose.orientation.z = _rotation.z();
    _marker.pose.orientation.w = _rotation.w();
    _marker.scale.x = _dimensions(0);
    _marker.scale.y = _dimensions(1);
    _marker.scale.z = _dimensions(2);

    if (_selected_cluster) {
        _marker.color.a = 0.9;

    } else {
        _marker.color.a = 0.45; 
    }

}

moveit_msgs::CollisionObject ObjectCluster::getCollisionObject(unsigned id) const{

    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = _cloud->header.frame_id;
    collision_object.header.stamp = ros::Time::now();
    collision_object.id = "box_cloud_" + std::to_string(id);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = _dimensions(0);
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = _dimensions(1);
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = _dimensions(2);
    collision_object.primitives.push_back(primitive);

    geometry_msgs::Pose pose;
    pose.position.x = _position (0); 
    pose.position.y = _position (1); 
    pose.position.z = _position (2); 
    pose.orientation.x = _rotation.x();
    pose.orientation.y = _rotation.y();
    pose.orientation.z = _rotation.z();
    pose.orientation.w = _rotation.w();

    collision_object.primitive_poses.push_back(pose);

    return collision_object;

}

void ObjectCluster::fillTransform(unsigned id) {
    
    _ref_T_cloud.header.stamp = ros::Time::now();
    _ref_T_cloud.header.frame_id = _cloud->header.frame_id;
    _ref_T_cloud.child_frame_id = "box_cloud_" + std::to_string(id);

    _ref_T_cloud.transform.translation.x = _position (0); 
    _ref_T_cloud.transform.translation.y = _position (1);
    _ref_T_cloud.transform.translation.z = _position (2);
    
    _ref_T_cloud.transform.rotation.x = _rotation.x();
    _ref_T_cloud.transform.rotation.y = _rotation.y();
    _ref_T_cloud.transform.rotation.z = _rotation.z();
    _ref_T_cloud.transform.rotation.w = _rotation.w();
}

geometry_msgs::PoseStamped ObjectCluster::getPose() const {

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _cloud->header.frame_id;
    pose.pose.position.x = _position (0); 
    pose.pose.position.y = _position (1); 
    pose.pose.position.z = _position (2); 
    pose.pose.orientation.x = _rotation.x();
    pose.pose.orientation.y = _rotation.y();
    pose.pose.orientation.z = _rotation.z();
    pose.pose.orientation.w = _rotation.w();

    return pose;
}

std::array<double,3> ObjectCluster::getDimensions() const {
    return {_dimensions(0), _dimensions(1), _dimensions(2)};
}

bool ObjectCluster::selectCluster(bool select) {
    
    _selected_cluster = select;
    return true;
}

bool ObjectCluster::findPoint(Point searchPoint, float radius) {
    
    _kdtree.setInputCloud (_cloud);

    if ( _kdtree.radiusSearch (searchPoint, radius, _point_idx_found, _point_radius_squared_distance, 1) > 0 )
    {
        // for (size_t i = 0; i < _point_idx_found.size (); ++i)
        //     std::cout << "    "  <<   _cloud->points[ _point_idx_found[0] ].x 
        //             << " " << _cloud->points[ _point_idx_found[0] ].y 
        //             << " " << _cloud->points[ _point_idx_found[0] ].z 
        //             << " (squared distance: " << _point_radius_squared_distance[0] << ")" << std::endl;
        //             std::cout << std::endl;
                
        _selected_cluster = true;

    } else {

        _selected_cluster = false;
    }
    
    return _selected_cluster;
}
