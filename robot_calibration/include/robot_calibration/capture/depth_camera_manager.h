/*
 * Copyright (C) 2015-2016 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
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

// Author: Michael Ferguson

#ifndef ROBOT_CALIBRATION_CAPTURE_DEPTH_CAMERA_H
#define ROBOT_CALIBRATION_CAPTURE_DEPTH_CAMERA_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/ExtendedCameraInfo.h>
#include <robot_calibration/capture/camera_manager_base.h>

namespace robot_calibration
{

  class DepthCameraManager final : public CameraManagerBase
  {
  public:
    DepthCameraManager();
    ~DepthCameraManager();

    std::string getFrameId() const override;
    ros::Time getStamp() const override;

    bool init(ros::NodeHandle &n) override;

    robot_calibration_msgs::ExtendedCameraInfo getExtendedCameraInfo() const override;

    cv::Mat_<cv::Vec3b> getRgbImage() override;

    bool solve2dTo3d(const std::vector<geometry_msgs::PointStamped> &object_points,
                     const std::vector<cv::Point2f> &image_coords, std::vector<geometry_msgs::PointStamped> &points) const override;

  private:
    void cameraInfoCallback(const sensor_msgs::CameraInfo::Ptr camera_info);

    void cameraCallback(const sensor_msgs::PointCloud2 &cloud);

    // Returns true if we got a message, false if we timeout
    bool waitForCloud();

    ros::Subscriber camera_info_subscriber_;
    bool camera_info_valid_;

    sensor_msgs::CameraInfo::Ptr camera_info_ptr_;

    double z_offset_mm_;
    double z_scaling_;

    bool waiting_;
    sensor_msgs::PointCloud2 cloud_;

    ros::Subscriber subscriber_; /// Incoming sensor_msgs::PointCloud2

  };

} // namespace robot_calibration

#endif // ROBOT_CALIBRATION_CAPTURE_DEPTH_CAMERA_H
