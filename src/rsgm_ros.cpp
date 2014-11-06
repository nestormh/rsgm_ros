/*
    Copyright 2014 Néstor Morales Hernández <nestor@isaatc.ull.es>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "rsgm_ros.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <tiff.h>

#include "rSGM.h"

namespace rsgm_ros {
    
RSGM_ROS::RSGM_ROS(const std::string& transport)
{
    ros::NodeHandle nh;
    int queue_size;
    
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 10);
    
    // Topics
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";
    
    image_transport::ImageTransport it(nh);
    m_left_sub.subscribe(it, left_topic, 1, transport);
    m_right_sub.subscribe(it, right_topic, 1, transport);
    m_left_info_sub.subscribe(nh, left_info_topic, 1);
    m_right_info_sub.subscribe(nh, right_info_topic, 1);
    
    // TODO: This is an initial attempt to access to the main code
    console(1, NULL);
    
    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
        m_approximate_sync.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                    m_left_sub, m_right_sub, m_left_info_sub, m_right_info_sub) );
        m_approximate_sync->registerCallback(boost::bind(&RSGM_ROS::process, this, _1, _2, _3, _4));
    }
    else
    {
        m_exact_sync.reset(new ExactSync(ExactPolicy(queue_size),
                                            m_left_sub, m_right_sub, m_left_info_sub, m_right_info_sub) );
        m_exact_sync->registerCallback(boost::bind(&RSGM_ROS::process, this, _1, _2, _3, _4));
    }
}    

void RSGM_ROS::process(const sensor_msgs::ImageConstPtr& l_image_msg, 
                       const sensor_msgs::ImageConstPtr& r_image_msg,
                       const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                       const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    cv_bridge::CvImageConstPtr leftImgPtr, rightImgPtr;
    leftImgPtr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    rightImgPtr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    m_model.fromCameraInfo(*l_info_msg, *r_info_msg);
    
    m_leftImage = leftImgPtr->image;
    m_rightImage = rightImgPtr->image;

    INIT_CLOCK(startCompute)
    
    cv::imshow("left", m_leftImage);
    cv::imshow("right", m_rightImage);
    
    uint8_t keycode;
    keycode = cv::waitKey(200);
    if (keycode == 27) {
        exit(0);
    }
    ros::spinOnce();
    
    END_CLOCK(totalCompute, startCompute)
    
    ROS_INFO("[%s] Total time: %f seconds", __FILE__, totalCompute);
        
}
}