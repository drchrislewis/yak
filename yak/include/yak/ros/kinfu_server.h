/*
 * KinfuServer.h
 *
 *  Created on: Jun 3, 2015
 *      Author: mklingen
 */

#ifndef KINFUSERVER_H_
#define KINFUSERVER_H_

#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <yak/kfusion/kinfu.hpp>
#include <yak/ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#include <yak/GetTSDF.h>
#include <yak/TSDF.h>
#include <yak/GetTSDFRequest.h>
#include <yak/GetTSDFResponse.h>

#include <yak/GetSparseTSDF.h>
#include <yak/SparseTSDF.h>
#include <yak/GetSparseTSDFRequest.h>
#include <yak/GetSparseTSDFResponse.h>

#include <yak/ResetVolume.h>
#include <yak/ResetVolumeRequest.h>
#include <yak/ResetVolumeResponse.h>

#include <yak/ros/half.hpp>
#include <tf/transform_listener.h>

#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>
#include <opencv2/core/eigen.hpp>

namespace kfusion
{
    
    class KinFuServer
    {
        public:
            KinFuServer(RosRGBDCamera* camera, const std::string& fixedFrame, const std::string& camFrame) ;

            template<typename T> void LoadParam(T& value, const std::string& name)
            {
                T newValue = camera_->nodeHandle.param<T>(name, value);
                value = newValue;
            }

            inline void LoadParam(float& value, const std::string& name)
            {
                double curr = value;
                curr = camera_->nodeHandle.param<double>(name, curr);
                value = static_cast<float>(curr);
                std::cout << name << ": " << value << std::endl;
            }

            // Should be called before running the server.
            inline void Initialize()
            {
                ConnectCamera();
                LoadParams();
            }

            // Does one full step of grabbing a camera image,
            // then tracking against the model, followed by
            // publishing relevant data to ROS.
            void Update();

            // Publish a raycasted image of the scene to ROS
            void PublishRaycastImage();
            // Connects to the camera, and runs until told to stop.
            bool ExecuteBlocking();
            // Connects to the ROS depth camera, blocking.
            bool ConnectCamera();
            // Creates the KinFu server from the ROS parameters
            bool LoadParams();
            // Publishes the current camera transform.
            bool PublishTransform();
            // Does a single KinFu step given a depth and (optional) color image.
            bool KinFu(const Affine3f& cameraMotionHint, const Affine3f& currentCameraPoseHint, const cv::Mat& depth, const cv::Mat& color);

             inline bool ShouldExit() const { return should_exit_; }
             inline void SetExit(bool value) { should_exit_ = value; }
             inline void Exit() { should_exit_ = true; }
             inline  KinFu::Ptr GetKinfu() { return kinfu_; }
             inline void SetKinfu(KinFu::Ptr value) { kinfu_ = value; }
             inline const RosRGBDCamera* GetCamera() { return camera_; }
             inline void SetCamera(RosRGBDCamera* value) { camera_ = value; }
             inline const cv::Mat& GetViewHost() { return viewHost_; }
             inline const cuda::Image& GetViewDevice() { return viewDevice_; }
             inline const cuda::Depth& GetDepthDevice() { return depthDevice_; }
             inline const std::string& GetBaseFrame() { return baseFrame_; }
             inline const std::string& GetCameraFrame() { return cameraFrame_; }

             // Service calls

             bool GetTSDF(yak::GetTSDFRequest& req, yak::GetTSDFResponse& res);

             bool GetSparseTSDF(yak::GetSparseTSDFRequest& req, yak::GetSparseTSDFResponse& res);

             bool TruncateTSDF(std::vector<uint32_t> &data, std::vector<uint32_t> &dataOut, std::vector<uint16_t> &rows, std::vector<uint16_t> &cols, std::vector<uint16_t> &sheets, int numVoxelsX, int numVoxelsY, int numVoxelsZ);

             bool GetTSDFData(uint32_t input,  half_float::half& voxelValue, uint16_t& voxelWeight);

             bool PublishCloud(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

             Affine3f TransformToAffine(tf::Transform input);

//             tf::Transform SwitchToVolumeFrame(tf::Transform input);

        protected:
            bool should_exit_;
            KinFu::Ptr kinfu_;
            RosRGBDCamera* camera_;
            cv::Mat viewHost_;
            cuda::Image viewDevice_;
            cuda::Depth depthDevice_;
            ros::Publisher raycastImgPublisher_;
            std::string baseFrame_;
            std::string cameraFrame_;
            tf::TransformBroadcaster tfBroadcaster_;
            tf::TransformListener tfListener_;
            ros::Publisher cloud_pub_;

            //tf::Transform camera_to_tool0_;

            tf::StampedTransform current_volume_to_sensor_transform_;
            tf::StampedTransform previous_volume_to_sensor_transform_;

            cv::Mat lastDepth_;
            cv::Mat lastColor_;
            Affine3f currentCameraMotionHint_;
            Affine3f currentCameraPoseHint_;

            ros::ServiceServer get_tsdf_server_;
            ros::ServiceServer get_sparse_tsdf_server_;
            ros::ServiceServer reset_volume_server_;
            ros::ServiceServer publish_cloud_server_;

            bool use_pose_hints_;
    };

} /* namespace kfusion */
#endif /* KINFUSERVER_H_ */
