/**
 * @file camera_manager_base.h
 * @author Thomas Lindemeier
 * @brief
 * @date 2020-11-10
 *
 */
#pragma once

#include <ros/ros.h>
#include <robot_calibration_msgs/ExtendedCameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace robot_calibration
{
  class CameraManagerBase
  {
  public:
    CameraManagerBase() = default;
    virtual ~CameraManagerBase() = default;

    CameraManagerBase(const CameraManagerBase &) = delete;
    CameraManagerBase &operator=(const CameraManagerBase &) = delete;

    virtual std::string getFrameId() const = 0;
    virtual ros::Time getStamp() const = 0;

    virtual bool init(ros::NodeHandle &n) = 0;

    virtual cv::Mat_<cv::Vec3b> getRgbImage() = 0;

    virtual robot_calibration_msgs::ExtendedCameraInfo getExtendedCameraInfo() const = 0;

    virtual bool solve2dTo3d(const std::vector<geometry_msgs::PointStamped> &object_points,
                             const std::vector<cv::Point2f> &image_coords, std::vector<geometry_msgs::PointStamped> &points) const = 0;
  };

} // namespace robot_calibration
