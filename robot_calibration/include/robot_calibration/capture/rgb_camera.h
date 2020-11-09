/**
 * @file rgb_camera.h
 * @author Thomas Lindemeier
 * @brief
 * @date 2020-11-09
 *
 */
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <robot_calibration_msgs/ExtendedCameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/imgproc/imgproc.hpp>
namespace robot_calibration
{

  class RgbCameraManager
  {
  public:
    RgbCameraManager() = default;
    ~RgbCameraManager() = default;

    RgbCameraManager(const RgbCameraManager &) = delete;
    RgbCameraManager &operator=(const RgbCameraManager &) = delete;

    bool init(ros::NodeHandle &n);

    cv::Mat_<cv::Vec3b> getRgbImage() const;

    robot_calibration_msgs::ExtendedCameraInfo getExtendedCameraInfo() const;

    void solve2dTo3d(const std::vector<cv::Vec3d> &object_points,
                     const std::vector<cv::Vec2f> &image_coords, std::vector<geometry_msgs::PointStamped>& points) const;

  private:
    ros::Subscriber camera_info_subscriber_;

    sensor_msgs::CameraInfoConstPtr camera_info_ptr_ = nullptr;

    image_transport::Subscriber image_subscriber_;

    sensor_msgs::ImageConstPtr image_ptr_ = nullptr;
  };

} // namespace robot_calibration
