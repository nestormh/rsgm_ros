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

#ifndef RSGM_ROS_H
#define RSGM_ROS_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>

#include <time.h>

using namespace std;

#define INIT_CLOCK(start) clock_t start = clock();
#define RESET_CLOCK(start) start = clock();
#define END_CLOCK(time, start) float time = (clock() - start) / (float)(CLOCKS_PER_SEC);
#define END_CLOCK_2(time, start) time = (clock() - start) / (float)(CLOCKS_PER_SEC);

namespace rsgm_ros {
    
class RSGM_ROS
{
public:
    RSGM_ROS(const std::string& transport);
    
protected:
    // Constants
    static const uint8_t LT0 = 0;
    static const uint8_t RT0 = 1;
    static const uint8_t RT1 = 2;
    static const uint8_t LT1 = 3;
    static const uint8_t LT0B = 4;
    
    // Type definitions
    typedef image_transport::SubscriberFilter Subscriber;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    
    // Callbacks
    void process(const sensor_msgs::ImageConstPtr& l_image_msg, 
                 const sensor_msgs::ImageConstPtr& r_image_msg,
                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                 const sensor_msgs::CameraInfoConstPtr& r_info_msg);
    
    // Properties
    cv::Mat m_leftImage, m_rightImage;

    image_geometry::StereoCameraModel m_model;

    Subscriber m_left_sub, m_right_sub;
    InfoSubscriber m_left_info_sub, m_right_info_sub;
    
    boost::shared_ptr<ExactSync> m_exact_sync;
    boost::shared_ptr<ApproximateSync> m_approximate_sync;
};

}

#endif // RSGM_ROS_H
