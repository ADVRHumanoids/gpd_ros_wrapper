/*
 * Copyright 2022 <copyright holder> <email>
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

/** Taken from tpo_vision, but reduced since now filtering is happening with the sensor filters */

#include <gpd_ros_wrapper/ExtractGraspPoses.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "ExtractGraspPoses" );
    ros::NodeHandle nh("~");
    
    ExtractGraspPoses extractGraspPoses(&nh);
    
    ros::Rate r(10);
    while(ros::ok()) {
        
       //extractGraspPoses.run();
       extractGraspPoses.run_no_tf_input();

        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
    
}
