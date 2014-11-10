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

// #include <time.h>
#include <omp.h>

#include "rSGM.h"


#include "rSGM/src/MyImage.h"

extern template class MyImage<uint8_t>;
extern uint32 readNumber(uint8* p, uint32& index);

using namespace std;

// #define INIT_CLOCK(start) clock_t start = clock();
// #define RESET_CLOCK(start) start = clock();
// #define END_CLOCK(time, start) float time = (clock() - start) / (float)(CLOCKS_PER_SEC);
// #define END_CLOCK_2(time, start) time = (clock() - start) / (float)(CLOCKS_PER_SEC);

#define INIT_CLOCK(start) double start = omp_get_wtime();
#define RESET_CLOCK(start) start = omp_get_wtime();
#define END_CLOCK(time, start) float time = omp_get_wtime() - start;
#define END_CLOCK_2(time, start) float time = omp_get_wtime() - start;

namespace rsgm_ros {
    
class RSGM_ROS
{
public:
    RSGM_ROS(const std::string& transport);
    
protected:
    enum Disparity_Method_t { METHOD_SGM = 0, METHOD_HCWS_CENSUS = 1 };
    
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
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> PointCloud;
    typedef uint8_t MyImage_Data_t;
    typedef MyImage<MyImage_Data_t> MyImage_t;
    
    // Callbacks
    void process(const sensor_msgs::ImageConstPtr& l_image_msg, 
                 const sensor_msgs::ImageConstPtr& r_image_msg,
                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                 const sensor_msgs::CameraInfoConstPtr& r_info_msg);
    
    
    
    void publish_point_cloud(const sensor_msgs::ImageConstPtr& l_image_msg, 
                             float32* l_disp_data,
                             const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                             const sensor_msgs::CameraInfoConstPtr& r_info_msg);
    
    MyImage_t fromCVtoMyImage(const cv::Mat & img);
    cv::Mat fromMyImagetoOpenCV(MyImage_t & myImg);
    
    // Properties
//     cv::Mat m_leftImage, m_rightImage;

    image_geometry::StereoCameraModel m_model;
    
    // Parameters
    uint32_t m_paths, m_threads, m_strips, m_dispCount;
    Disparity_Method_t m_disparityMethod;
    Sampling_Method_t m_samplingMethod;
    bool m_downsample;

    Subscriber m_left_sub, m_right_sub;
    InfoSubscriber m_left_info_sub, m_right_info_sub;
    ros::Publisher m_pointCloud;
    
    boost::shared_ptr<ExactSync> m_exact_sync;
    boost::shared_ptr<ApproximateSync> m_approximate_sync;
};

}

#endif // RSGM_ROS_H
