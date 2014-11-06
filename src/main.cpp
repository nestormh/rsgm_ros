/*
 *  Copyright 2014 Néstor Morales Hernández <nestor@isaatc.ull.es>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <iostream>
#include "rsgm_ros.h"

#include <ros/ros.h>

using namespace std;
using namespace rsgm_ros;

int main(int argC, char **argV) {
    ros::init(argC, argV, "oflow_3d_generator");
    
    std::string transport = argC > 1 ? argV[1] : "raw";
    RSGM_ROS rsgm(transport);
    
    ros::spin();
    
    return 0;
}