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

#include "rSGM/src/MyImage.hpp"
#include "rSGM/src/rSGMCmd.cpp"

const static string DISPARITY_NAMES_SGM_5x5 = "sgm_5x5";
const static string DISPARITY_NAMES_HCWS_CENSUS_9x7 = "hwcs_census_9x7";

const static string SAMPLING_NAMES_STANDARD_SGM = "standard_sgm";
const static string SAMPLING_NAMES_STRIPED_SGM = "striped_sgm";
const static string SAMPLING_NAMES_STRIPED_SGM_SUBSAMPLE_2 = "striped_sgm_subsample_2";
const static string SAMPLING_NAMES_STRIPED_SGM_SUBSAMPLE_4 = "striped_sgm_subsample_4";

namespace rsgm_ros {
    
RSGM_ROS::RSGM_ROS(const std::string& transport)
{
    ros::NodeHandle nh;
    int queue_size;
    string dispMethod, samplingMethod;
    int paths, threads, strips, dispCount;
    
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 10);
    
    local_nh.param("paths", paths, 8);
    local_nh.param("threads", threads, 8);
    local_nh.param("strips", strips, 8);
    local_nh.param("disp_count", dispCount, 64);
    
    m_paths = paths;
    m_threads = threads;
    m_strips = strips;
    m_dispCount = dispCount;
    
    local_nh.param("disparity_method", dispMethod, DISPARITY_NAMES_HCWS_CENSUS_9x7);
    local_nh.param("sampling_method", samplingMethod, SAMPLING_NAMES_STRIPED_SGM);
    
    if (dispMethod == DISPARITY_NAMES_SGM_5x5) {
        m_disparityMethod = METHOD_SGM;
    } else if (dispMethod == DISPARITY_NAMES_HCWS_CENSUS_9x7) {
        m_disparityMethod = METHOD_HCWS_CENSUS;
    } else {
        ROS_ERROR("disparity_method %s not valid!. Try: [%s, %s]", 
                  dispMethod.c_str(), DISPARITY_NAMES_SGM_5x5.c_str(), DISPARITY_NAMES_HCWS_CENSUS_9x7.c_str());
        ROS_INFO("Aborting...");
        exit(0);
    }
    
    if (samplingMethod == SAMPLING_NAMES_STANDARD_SGM) {
        m_samplingMethod = STANDARD_SGM;
    } else if (samplingMethod == SAMPLING_NAMES_STRIPED_SGM) {
        m_samplingMethod = STRIPED_SGM;
    } else if (samplingMethod == SAMPLING_NAMES_STRIPED_SGM_SUBSAMPLE_2) {
        m_samplingMethod = STRIPED_SGM_SUBSAMPLE_2;
    } else if (samplingMethod == SAMPLING_NAMES_STRIPED_SGM_SUBSAMPLE_4) {
        m_samplingMethod = STRIPED_SGM_SUBSAMPLE_4;
    } else {
        ROS_ERROR("sampling_method %s not valid!. Try: [%s, %s, %s. %s]", 
                  samplingMethod.c_str(), SAMPLING_NAMES_STANDARD_SGM.c_str(), SAMPLING_NAMES_STRIPED_SGM.c_str(),
                  SAMPLING_NAMES_STRIPED_SGM_SUBSAMPLE_2.c_str(), SAMPLING_NAMES_STRIPED_SGM_SUBSAMPLE_4.c_str());
        ROS_INFO("Aborting...");
        exit(0);
    }
    
    if (m_threads != 2 && m_threads != 4 && m_threads != 8) {
        ROS_ERROR("The number of threads is %d", m_threads);
        ROS_ERROR("It should be 2, 4 or 8!!!");
        ROS_INFO("Aborting...");
        exit(0);
    }

    if ((m_strips / m_threads == 0) || (m_strips % m_threads != 0)) {
        ROS_ERROR("The number of strips can not be smaller than the number of threads!!!");
        ROS_ERROR("Division between the number of strips and threads should be exact!!!");
        ROS_ERROR("Number of threads: %d", m_threads);
        ROS_ERROR("Number of strips: %d", m_strips);
        ROS_INFO("Aborting...");
        exit(0);
    }
    
    ROS_INFO("PARAMS INFO");
    ROS_INFO("===========");
    ROS_INFO("paths: %d", m_paths);
    ROS_INFO("threads: %d", m_threads);
    ROS_INFO("strips: %d", m_strips);
    ROS_INFO("disp_count: %d", m_dispCount);
    ROS_INFO("disparity_method: %s", dispMethod.c_str());
    ROS_INFO("sampling_method: %s", samplingMethod.c_str());
    
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
    INIT_CLOCK(startCompute)
    cv_bridge::CvImageConstPtr leftImgPtr, rightImgPtr;
    leftImgPtr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    rightImgPtr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    m_model.fromCameraInfo(*l_info_msg, *r_info_msg);
    
    MyImage_t myImgLeft = fromCVtoMyImage(leftImgPtr->image);
    MyImage_t myImgRight = fromCVtoMyImage(rightImgPtr->image);
    
    MyImage<uint8> disp(myImgLeft.getWidth(), myImgLeft.getHeight());
    
    // start processing
    float32* dispImgLeft = (float32*)_mm_malloc(myImgLeft.getWidth()*myImgLeft.getHeight()*sizeof(float32), 16);
    float32* dispImgRight = (float32*)_mm_malloc(myImgLeft.getWidth()*myImgLeft.getHeight()*sizeof(float32), 16);
    
    uint8* leftImg = myImgLeft.getData();
    uint8* rightImg = myImgRight.getData();
    
//     enum Disparity_Method_t { METHOD_SGM = 0, METHOD_HCWS_CENSUS = 1 };
    switch (m_disparityMethod) {
        case METHOD_SGM: {
            processCensus5x5SGM(leftImg, rightImg, dispImgLeft, dispImgRight, 
                                myImgLeft.getWidth(), myImgLeft.getHeight(),
                                m_disparityMethod, m_paths, m_threads, m_strips, m_dispCount);
            break;
        }
        case METHOD_HCWS_CENSUS: {
            processCensus9x7SGM(leftImg, rightImg, dispImgLeft, dispImgRight, 
                                myImgLeft.getWidth(), myImgLeft.getHeight(),
                                m_disparityMethod, m_paths, m_threads, m_strips, m_dispCount);
            break;
        }
        default: {
            ROS_ERROR("[At %s:%d] This message should never be shown. Aborting...", __FILE__, __LINE__);
        }
    }
    
    // write output
    uint8* dispOut = disp.getData();
    for (uint32 i = 0; i < myImgLeft.getWidth()*myImgLeft.getHeight(); i++) {
        if (dispImgLeft[i]>0) {
            dispOut[i] = (uint8)dispImgLeft[i];
        }
        else {
            dispOut[i] = 0;
        }
    }
    
    END_CLOCK(totalCompute, startCompute)
    ROS_INFO("[%s] Total time: %f seconds", __FILE__, totalCompute);
    
    cv::Mat dispMap = fromMyImagetoOpenCV(disp);
    
    double min = 0;
    double max = m_dispCount;
//     cv::minMaxIdx(dispMap, &min, &max);
    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    dispMap.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 
    cout << "min " << min << endl;
    cout << "max " << max << endl;
    
    // this is great. It converts your grayscale image into a tone-mapped one, 
    // much more pleasing for the eye
    // function is found in contrib module, so include contrib.hpp 
    // and link accordingly
    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
    
    cv::imshow("Out", falseColorsMap);
    
//     writePGM(disp, "/tmp/disp.pgm", true);

//     cv::imshow("left", leftImg2);
    uint8_t keycode;
    keycode = cv::waitKey(200);
    if (keycode == 27) {
        exit(0);
    }
    ros::spinOnce();
        
}

RSGM_ROS::MyImage_t RSGM_ROS::fromCVtoMyImage(const cv::Mat& img)
{
    MyImage_t myImg;
    const uint32 & width = img.cols;
    const uint32 & height = img.rows;
    
    // copy values
    MyImage_Data_t* data = (MyImage_Data_t*)_mm_malloc(width * height * sizeof(MyImage_Data_t), 16);
    memcpy(data, img.data, width*height*sizeof(MyImage_Data_t));
    
    myImg.setAttributes(width, height, data);
    
    return myImg;
}

cv::Mat RSGM_ROS::fromMyImagetoOpenCV(RSGM_ROS::MyImage_t& myImg)
{
    const uint32 width = myImg.getWidth();
    const uint32 height = myImg.getHeight();
    MyImage_Data_t* data = myImg.getData();
    
    cv::Mat img(height, width, CV_8UC1);
    
    memcpy(img.data, data, width * height * sizeof(MyImage_Data_t));
    
    return img;
}

}