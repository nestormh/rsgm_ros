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
#include <boost/graph/graph_concepts.hpp>

#include "rSGM/src/MyImage.hpp"
#include "rSGM/src/rSGMCmd.cpp"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

#include <stereo_msgs/DisparityImage.h>

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
    int threads, strips, dispCount;
    string depthEncoding;
    
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 10);
    
    local_nh.param("threads", threads, 8);
    local_nh.param("strips", strips, 8);
    local_nh.param("disp_count", dispCount, 64);
    local_nh.param("downsample", m_downsample, false);
    
    m_threads = threads;
    m_strips = strips;
    m_dispCount = dispCount;
    
    local_nh.param("lr_check", m_params.lrCheck, true);
    local_nh.param("median_filter", m_params.MedianFilter, false);
    local_nh.param("paths", m_params.Paths, 8);
    local_nh.param("sub_pixel_refine", m_params.subPixelRefine, 0);
    local_nh.param("number_of_passes", m_params.NoPasses, 2);
    local_nh.param("rl_check", m_params.rlCheck, false);
    local_nh.param("invalid_disparity_cost", m_params.InvalidDispCost, 16);
    local_nh.param("gamma", m_params.Gamma, 100);
    local_nh.param("alpha", m_params.Alpha, 0.0);
    local_nh.param("p1", m_params.P1, 10);
    local_nh.param("p2", m_params.P2min, 50);
    local_nh.param("uniqueness", m_params.Uniqueness, 0.90);

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
    
    m_pointCloudPub = local_nh.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);
    
    image_transport::ImageTransport local_it(local_nh);
    m_disparityImagePub = local_it.advertise("disparity", 1);
    m_depthImagePub = local_it.advertise("depth", 1);
    
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
    
    ROS_INFO("PARAMS INFO");
    ROS_INFO("===========");
    ROS_INFO("threads: %d", m_threads);
    ROS_INFO("strips: %d", m_strips);
    ROS_INFO("disp_count: %d", m_dispCount);
    ROS_INFO("disparity_method: %s", dispMethod.c_str());
    ROS_INFO("sampling_method: %s", samplingMethod.c_str());
    ROS_INFO("lr_check %d", m_params.lrCheck);
    ROS_INFO("median_filter %d", m_params.MedianFilter);
    ROS_INFO("paths %d", m_params.Paths);
    ROS_INFO("sub_pixel_refine %d", m_params.subPixelRefine);
    ROS_INFO("number_of_passes %d", m_params.NoPasses);
    ROS_INFO("rl_check %d", m_params.rlCheck);
    ROS_INFO("invalid_disparity_cost %d", m_params.InvalidDispCost);
    ROS_INFO("gamma %d", m_params.Gamma);
    ROS_INFO("alpha %f", m_params.Alpha);
    ROS_INFO("p1 %d", m_params.P1);
    ROS_INFO("p2 %d", m_params.P2min);
    ROS_INFO("uniqueness %f", m_params.Uniqueness);
    ROS_INFO("approximate_sync %d", approx);
    ROS_INFO("queue_size %d", queue_size);
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
                                m_disparityMethod, m_params.Paths, m_threads, m_strips, m_dispCount);
            break;
        }
        case METHOD_HCWS_CENSUS: {
            StereoSGMParams_t params(m_params.P1, m_params.InvalidDispCost, m_params.NoPasses, m_params.Paths,
                                     m_params.Uniqueness, m_params.MedianFilter, m_params.lrCheck, m_params.rlCheck,
                                     m_params.lrThreshold, m_params.subPixelRefine, m_params.Alpha, m_params.Gamma,
                                     m_params.P2min);
            
            processCensus9x7SGM(leftImg, rightImg, dispImgLeft, dispImgRight, 
                                myImgLeft.getWidth(), myImgLeft.getHeight(),
                                m_disparityMethod, params.Paths, m_threads, m_strips, m_dispCount, params);
            break;
        }
        default: {
            ROS_ERROR("[At %s:%d] This message should never be shown. Aborting...", __FILE__, __LINE__);
        }
    }
    
    INIT_CLOCK(startPointCloudPublish)
    publish_point_cloud(l_image_msg, dispImgLeft, l_info_msg, r_info_msg);
    END_CLOCK(totalPointCloudPublish, startPointCloudPublish)
    INIT_CLOCK(startDispImgPublish)
    publishDisparityMap(l_image_msg, dispImgLeft, l_info_msg, r_info_msg);
    END_CLOCK(totalDispImgPublish, startDispImgPublish)
    
    END_CLOCK(totalCompute, startCompute)
    ROS_INFO("[%s] Publishing point cloud time: %f seconds", __FILE__, totalPointCloudPublish);
    ROS_INFO("[%s] Publishing Disparity Image time: %f seconds", __FILE__, totalDispImgPublish);
    ROS_INFO("[%s] Total time: %f seconds", __FILE__, totalCompute);
}

RSGM_ROS::MyImage_t RSGM_ROS::fromCVtoMyImage(const cv::Mat& img)
{
    MyImage_t myImg;
    uint32 width = img.cols;
    uint32 height = img.rows;
    
    if (width % 4 != 0) {
        width = floor(width / 4) * 4;
    } 
    cv::Mat roi(img, cv::Rect(0, 0, width, height));
    
    // copy values
    MyImage_Data_t* data = (MyImage_Data_t*)_mm_malloc(width * height * sizeof(MyImage_Data_t), 16);
    memcpy(data, roi.data, width*height*sizeof(MyImage_Data_t));
    
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

void RSGM_ROS::publish_point_cloud(const sensor_msgs::ImageConstPtr& l_image_msg, 
                                   float32* l_disp_data,
                                   const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                                   const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
//     if (m_pointCloudPub.getNumSubscribers() == 0)
//         return;
    
    try
    {
        const cv::Mat leftImg = (cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8))->image;
        
        const int32_t & l_width = leftImg.cols;
        const int32_t & l_height = leftImg.rows;
        
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(*l_info_msg, *r_info_msg);
        
        PointCloud::Ptr point_cloud(new PointCloud());
        point_cloud->width = 1;
        point_cloud->points.reserve(l_width * l_height);
        
        uint32_t dSample = 1;
        if (m_downsample)
            dSample = 2;
        
        for (uint32_t u = 0; u < l_width; u += dSample) {
            for (uint32_t v = 0; v < l_height; v += dSample) {
                uint32_t index = v * l_width + u;
                
                if (l_disp_data[index] > 0) {
                    
                    cv::Point2d left_uv;
                    left_uv.x = u;
                    left_uv.y = v;

                    cv::Point3d point;
                    model.projectDisparityTo3d(left_uv, l_disp_data[index], point);
                    
                    PointType pointPCL;
                    
                    pointPCL.x = point.x;
                    pointPCL.y = point.y;
                    pointPCL.z = point.z;
                    const cv::Vec3b & pointColor = leftImg.at<cv::Vec3b>(v, u);
                    pointPCL.r = pointColor[0];
                    pointPCL.g = pointColor[1];
                    pointPCL.b = pointColor[2];
                    
                    point_cloud->push_back(pointPCL);
                }
            }
        }
        
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg (*point_cloud, cloudMsg);
        cloudMsg.header.frame_id = l_info_msg->header.frame_id;
        cloudMsg.header.stamp = ros::Time::now();
        cloudMsg.header.seq = l_info_msg->header.seq;
        
        m_pointCloudPub.publish(cloudMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RSGM_ROS::publishDisparityMap(const sensor_msgs::ImageConstPtr& imageMsg, float32 * dispData,
                                   const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                                   const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    bool publishDisparity = (m_disparityImagePub.getNumSubscribers() != 0);
    bool publishDepth = (m_depthImagePub.getNumSubscribers() != 0);
    
    if ((! publishDisparity) && (! publishDepth))
        return;
    
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);
    
    const cv::Mat leftImg = (cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::RGB8))->image;
    
    const int32_t & width = leftImg.cols;
    const int32_t & height = leftImg.rows;
    
    cv_bridge::CvImage disparityMsg, depthMsg;
    if (publishDisparity) {
        disparityMsg.header = imageMsg->header;
        disparityMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        disparityMsg.image = cv::Mat(height, width, CV_32FC1);
    }
    if (publishDepth) {
        depthMsg.header = imageMsg->header;
        depthMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depthMsg.image = cv::Mat(height, width, CV_32FC1);
    }

    #pragma omp num_threads(m_threads)
    for (uint32_t v = 0, i = 0; v < height; v++)  {
        for (uint32_t u = 0; u < width; u++, i++)  {
            if (dispData[i] > 0) {
                if (publishDisparity) disparityMsg.image.at<float32>(v, u) = std::max(dispData[i], 0.0f);
                if (publishDepth) depthMsg.image.at<float32>(v, u) = std::max(model.getZ(dispData[i]), 0.0);
            }
        }
    }
    
    // Publish
    if (publishDisparity) m_disparityImagePub.publish(disparityMsg.toImageMsg());
    if (publishDepth) m_depthImagePub.publish(depthMsg.toImageMsg());
}


}

