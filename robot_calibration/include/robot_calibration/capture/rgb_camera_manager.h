/**
 * @file rgb_camera_manager.h
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
#include <robot_calibration/capture/camera_manager_base.h>

#include <opencv2/imgproc/imgproc.hpp>
namespace robot_calibration
{

  class RgbCameraManager final : public CameraManagerBase
  {
  public:
    RgbCameraManager() = default;
    ~RgbCameraManager() = default;

    RgbCameraManager(const RgbCameraManager &) = delete;
    RgbCameraManager &operator=(const RgbCameraManager &) = delete;

    std::string getFrameId() const override;
    ros::Time getStamp() const override;

    bool init(ros::NodeHandle &n) override;

    cv::Mat_<cv::Vec3b> getRgbImage() override;

    robot_calibration_msgs::ExtendedCameraInfo getExtendedCameraInfo() const override;

    bool solve2dTo3d(const std::vector<geometry_msgs::PointStamped> &object_points,
                     const std::vector<cv::Point2f> &image_coords, std::vector<geometry_msgs::PointStamped> &points) const override;

  private:
    ros::Subscriber camera_info_subscriber_;

    sensor_msgs::CameraInfoConstPtr camera_info_ptr_ = nullptr;

    image_transport::Subscriber image_subscriber_;

    sensor_msgs::ImageConstPtr image_ptr_ = nullptr;

    bool waiting_ = false;
  };

} // namespace robot_calibration
