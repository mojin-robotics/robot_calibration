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
#include <robot_calibration/capture/depth_camera.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace robot_calibration
{
  // We use a number of PC2 iterators, define the indexes here
  const unsigned X = 0;
  const unsigned Y = 1;
  const unsigned Z = 2;

  DepthCameraManager::DepthCameraManager() : camera_info_valid_(false), waiting_(false) {}

  DepthCameraManager::~DepthCameraManager() {}

  std::string DepthCameraManager::getFrameId() const
  {
    return cloud_.header.frame_id;
  }

  ros::Time DepthCameraManager::getStamp() const
  {
    return cloud_.header.stamp;
  }

  bool DepthCameraManager::init(ros::NodeHandle &n)
  {
    std::string topic_name;
    n.param<std::string>("sensor_info_topic", topic_name, "/head_camera/depth/camera_info");

    camera_info_subscriber_ = n.subscribe(topic_name,
                                          1,
                                          &DepthCameraManager::cameraInfoCallback,
                                          this);

    // Setup Subscriber
    std::string topic_info_name;
    n.param<std::string>("sensor_data_topic", topic_info_name, "/points");
    subscriber_ = n.subscribe(topic_info_name, 1, &DepthCameraManager::cameraCallback, this);

    // Get parameters of drivers
    std::string driver_name;
    n.param<std::string>("camera_driver", driver_name, "/head_camera/driver");
    if (!n.getParam(driver_name + "/z_offset_mm", z_offset_mm_) ||
        !n.getParam(driver_name + "/z_scaling", z_scaling_))
    {
      ROS_ERROR("%s is not set, are drivers running?", driver_name.c_str());
      z_offset_mm_ = 0;
      z_scaling_ = 1;
    }

    // Wait for camera_info
    int count = 25;
    while (--count)
    {
      if (camera_info_valid_)
      {
        return true;
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    ROS_WARN("CameraInfo receive timed out.");
    return false;
  }

  robot_calibration_msgs::ExtendedCameraInfo DepthCameraManager::getExtendedCameraInfo() const
  {
    robot_calibration_msgs::ExtendedCameraInfo info;
    info.camera_info = *camera_info_ptr_;
    info.parameters.resize(2);
    info.parameters[0].name = "z_offset_mm";
    info.parameters[0].value = z_offset_mm_;
    info.parameters[1].name = "z_scaling";
    info.parameters[1].value = z_scaling_;
    return info;
  }

  cv::Mat_<cv::Vec3b> DepthCameraManager::getRgbImage()
  {
    // Get cloud
    if (!waitForCloud())
    {
      ROS_ERROR("No point cloud data");
      return cv::Mat_<cv::Vec3b>();
    }

    if (cloud_.height == 1)
    {
      ROS_ERROR("OpenCV does not support unorganized cloud/image.");
      return cv::Mat_<cv::Vec3b>();
    }

    if ((cloud_.height + cloud_.width) <= 1U)
    {
      return cv::Mat_<cv::Vec3b>();
    }

    cv::Mat_<cv::Vec3b> image(cloud_.height, cloud_.width);
    sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(cloud_, "rgb");
    for (size_t y = 0; y < cloud_.height; y++)
    {
      for (size_t x = 0; x < cloud_.width; x++)
      {
        image(y, x)[0U] = rgb[0U];
        image(y, x)[1U] = rgb[1U];
        image(y, x)[2U] = rgb[2U];
        ++rgb;
      }
    }
    return image;
  }

  bool DepthCameraManager::solve2dTo3d(const std::vector<geometry_msgs::PointStamped> &object_points,
                                       const std::vector<cv::Point2f> &image_coords, std::vector<geometry_msgs::PointStamped> &points) const
  {
    geometry_msgs::PointStamped rgbd;

    // Fill in the headers
    rgbd.header = cloud_.header;

    points.resize(image_coords.size());

    // Fill in message
    sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud_, "x");
    for (size_t i = 0; i < image_coords.size(); ++i)
    {
      // Get 3d point
      size_t index = static_cast<size_t>(image_coords[i].y) * cloud_.width +
                     static_cast<size_t>(image_coords[i].x);
      rgbd.point.x = (xyz + index)[X];
      rgbd.point.y = (xyz + index)[Y];
      rgbd.point.z = (xyz + index)[Z];

      // Do not accept NANs
      if (std::isnan(rgbd.point.x) || std::isnan(rgbd.point.y) || std::isnan(rgbd.point.z))
      {
        ROS_ERROR_STREAM("CheckerBoardDetector:: Rejecting observation due to NAN point on " << i);
        return false;
      }

      // Do not accept (0.0, 0.0, 0.0)
      const double eps = static_cast<double>(10.0) * std::numeric_limits<double>::epsilon();
      if ((std::fabs(rgbd.point.x) < eps) || (std::fabs(rgbd.point.y) < eps) || (std::fabs(rgbd.point.z) < eps))
      {
        ROS_ERROR_STREAM("CheckerBoardDetector:: Rejecting observation due to (0.0, 0.0, 0.0) point on " << i);
        return false;
      }

      points[i] = rgbd;
    }
    return true;
  }

  void DepthCameraManager::cameraInfoCallback(const sensor_msgs::CameraInfo::Ptr camera_info)
  {
    camera_info_ptr_ = camera_info;
    camera_info_valid_ = true;
  }

  void DepthCameraManager::cameraCallback(const sensor_msgs::PointCloud2 &cloud)
  {
    if (waiting_)
    {
      cloud_ = cloud;
      waiting_ = false;
    }
  }

  // Returns true if we got a message, false if we timeout
  bool DepthCameraManager::waitForCloud()
  {
    // Initial wait cycle so that camera is definitely up to date.
    ros::Duration(1 / 10.0).sleep();

    waiting_ = true;
    int count = 250;
    while (--count && ros::ok())
    {
      if (!waiting_)
      {
        // success
        return true;
      }
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
    ROS_ERROR("Failed to get cloud");
    return !waiting_;
  }

} // namespace robot_calibration
