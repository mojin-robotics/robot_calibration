/**
 * @file rgb_camera.cpp
 * @author Thomas Lindemeier
 * @brief
 * @date 2020-11-09
 *
 */
#include <robot_calibration/capture/rgb_camera.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d/calib3d.hpp>

namespace robot_calibration
{

  bool RgbCameraManager::init(ros::NodeHandle &n)
  {
    std::string info_topic_name;
    n.param<std::string>("rgb_camera_info_topic", info_topic_name, "/camera_info");
    camera_info_subscriber_ = n.subscribe<
        sensor_msgs::CameraInfo, const sensor_msgs::CameraInfoConstPtr &>(
        info_topic_name, 1,
        [this](const sensor_msgs::CameraInfoConstPtr &camera_info_ptr) {
          camera_info_ptr_ = camera_info_ptr;
        });

    image_transport::ImageTransport image_transport(n);
    std::string image_topic_name;
    n.param<std::string>("image_topic_name", image_topic_name, "/image");
    image_subscriber_ = image_transport.subscribe(
        image_topic_name, 1,
        [this](const sensor_msgs::ImageConstPtr &image_ptr) {
          image_ptr_ = image_ptr;
        });

    // Wait for camera_info
    int count = 25;
    while (--count)
    {
      if (camera_info_ptr_ != nullptr)
      {
        return true;
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    ROS_WARN("CameraInfo received timed out.");
    return false;
  }

  cv::Mat_<cv::Vec3b> RgbCameraManager::getRgbImage() const
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return cv::Mat_<cv::Vec3b>();
    }

    return cv_ptr->image;
  }

  robot_calibration_msgs::ExtendedCameraInfo RgbCameraManager::getExtendedCameraInfo() const
  {
    robot_calibration_msgs::ExtendedCameraInfo info;
    info.camera_info = *camera_info_ptr_;
    info.parameters.resize(2);
    info.parameters[0].name = "z_offset_mm";
    info.parameters[0].value = 0.0;
    info.parameters[1].name = "z_scaling";
    info.parameters[1].value = 1.0;
    return info;
  }

  void RgbCameraManager::solve2dTo3d(const std::vector<cv::Vec3d> &object_points,
                                     const std::vector<cv::Vec2f> &image_coords, std::vector<geometry_msgs::PointStamped> &points) const
  {
    points.clear();

    if (camera_info_ptr_ == nullptr)
    {
      ROS_ERROR("no camera info availabe");
      return;
    }

    cv::Mat1d matrix3x3 = cv::Mat1d(3U, 3U);
    for (auto i = 0U; i < matrix3x3.total(); i++)
    {
      matrix3x3(i) = camera_info_ptr_->K[i];
    }

    const auto dist_coeffs = camera_info_ptr_->D;
    auto rvec = cv::Mat1d(3U, 1U);
    auto tvec = cv::Mat1d(3U, 1U);

    // only we have valid camera info, equally sized list of points and succesfull call to cv::solvePnP
    if ((image_coords.size() == object_points.size()) &&
        cv::solvePnP(object_points, image_coords, matrix3x3, dist_coeffs, rvec,
                     tvec, false, cv::SOLVEPNP_ITERATIVE))
    {
      cv::Mat1d rotation_matrix = cv::Mat1d(3U, 3U);
      // axis angle to matrix
      cv::Rodrigues(rvec, rotation_matrix);

      // Construct affine transformation consisting of translation and rotation.
      cv::Affine3d pose(rotation_matrix, tvec);
      // apply the transformation to each point
      geometry_msgs::PointStamped xyz_stamped;
      xyz_stamped.header = image_ptr_->header;
      for (const auto coord : object_points)
      {
        const auto p = pose * coord;
        xyz_stamped.point.x = p[0U];
        xyz_stamped.point.y = p[1U];
        xyz_stamped.point.z = p[2U];
        points.push_back(xyz_stamped);
      }
    }
  }

} // namespace robot_calibration
