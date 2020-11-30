/*********************************************************************
 * Took the original processor.h file and made this for the 
 * last part of processing.
 *********************************************************************/

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/assert.h>
#include <sensor_msgs/image_encodings.h>
#include "stereo_image_proc/processor.h"

namespace stereo_image_proc {

  inline bool isValidPoint(const cv::Vec3f& pt)
  {
	  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
	  // and zero disparities (point mapped to infinity).
	  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
  }


  static void points2Processing(const stereo_msgs::DisparityImage& disparity,
                      const cv::Mat& color, const std::string& encoding,
                      const image_geometry::StereoCameraModel& model,
                      sensor_msgs::PointCloud2& points, cv::Mat_<cv::Vec3f> dense_points)
		 
  {
	  // Calculate dense point cloud
  const sensor_msgs::Image& dimage = disparity.image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model.projectDisparityImageTo3d(dmat, dense_points, true);

  // Fill in sparse point cloud message
  points.height = dense_points.rows;
  points.width  = dense_points.cols;
  points.fields.resize (4);
  points.fields[0].name = "x";
  points.fields[0].offset = 0;
  points.fields[0].count = 1;
  points.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[1].name = "y";
  points.fields[1].offset = 4;
  points.fields[1].count = 1;
  points.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[2].name = "z";
  points.fields[2].offset = 8;
  points.fields[2].count = 1;
  points.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[3].name = "rgb";
  points.fields[3].offset = 12;
  points.fields[3].count = 1;
  points.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  //points.is_bigendian = false; ???
  points.point_step = 16;
  points.row_step = points.point_step * points.width;
  points.data.resize (points.row_step * points.height);
  points.is_dense = false; // there may be invalid points

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int i = 0;
  for (int32_t u = 0; u < dense_points.rows; ++u) {
    for (int32_t v = 0; v < dense_points.cols; ++v, ++i) {
      if (isValidPoint(dense_points(u,v))) {
        // x,y,z,rgba
        memcpy (&points.data[i * points.point_step + 0], &dense_points(u,v)[0], sizeof (float));
        memcpy (&points.data[i * points.point_step + 4], &dense_points(u,v)[1], sizeof (float));
        memcpy (&points.data[i * points.point_step + 8], &dense_points(u,v)[2], sizeof (float));
      }
      else {
        memcpy (&points.data[i * points.point_step + 0], &bad_point, sizeof (float));
        memcpy (&points.data[i * points.point_step + 4], &bad_point, sizeof (float));
        memcpy (&points.data[i * points.point_step + 8], &bad_point, sizeof (float));
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  i = 0;
  if (encoding == enc::MONO8) {
    for (int32_t u = 0; u < dense_points.rows; ++u) {
      for (int32_t v = 0; v < dense_points.cols; ++v, ++i) {
        if (isValidPoint(dense_points(u,v))) {
          uint8_t g = color.at<uint8_t>(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          memcpy (&points.data[i * points.point_step + 12], &rgb, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::RGB8) {
    for (int32_t u = 0; u < dense_points.rows; ++u) {
      for (int32_t v = 0; v < dense_points.cols; ++v, ++i) {
        if (isValidPoint(dense_points(u,v))) {
          const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          memcpy (&points.data[i * points.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::BGR8) {
    for (int32_t u = 0; u < dense_points.rows; ++u) {
      for (int32_t v = 0; v < dense_points.cols; ++v, ++i) {
        if (isValidPoint(dense_points(u,v))) {
          const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          memcpy (&points.data[i * points.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
        else {
          memcpy (&points.data[i * points.point_step + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else {
    ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
  }
  }

} //namespace stereo_image_proc

