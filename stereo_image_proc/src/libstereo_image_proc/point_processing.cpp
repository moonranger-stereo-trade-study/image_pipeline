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


  static void processPoints(const stereo_msgs::DisparityImage& disparity,
                     const cv::Mat& color, const std::string& encoding,
                     const image_geometry::StereoCameraModel& model,
                     sensor_msgs::PointCloud& points, cv::Mat_<cv::Vec3f> dense_points)
  {
	 // Calculate dense point cloud
	 const sensor_msgs::Image& dimage = disparity.image;
	 const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
	 model.projectDisparityImageTo3d(dmat, dense_points, true);

	 // Fill in sparse point cloud message
	 points.points.resize(0);
	 points.channels.resize(3);
	 points.channels[0].name = "rgb";
	 points.channels[0].values.resize(0);
	 points.channels[1].name = "u";
	 points.channels[1].values.resize(0);
	 points.channels[2].name = "v";
	 points.channels[2].values.resize(0);

	 for (int32_t u = 0; u < dense_points.rows; ++u) {
	   for (int32_t v = 0; v < dense_points.cols; ++v) {
	     if (isValidPoint(dense_points(u,v))) {
	       // x,y,z
	        geometry_msgs::Point32 pt;
        	pt.x = dense_points(u,v)[0];
	        pt.y = dense_points(u,v)[1];
        	pt.z = dense_points(u,v)[2];
	        points.points.push_back(pt);
	        // u,v
	        points.channels[1].values.push_back(u);
	        points.channels[2].values.push_back(v);
	      }
	    }
	  }

	  // Fill in color
	  namespace enc = sensor_msgs::image_encodings;
	  points.channels[0].values.reserve(points.points.size());
	  if (encoding == enc::MONO8) {
	    for (int32_t u = 0; u < dense_points.rows; ++u) {
	      for (int32_t v = 0; v < dense_points.cols; ++v) {
	        if (isValidPoint(dense_points(u,v))) {
        	  uint8_t g = color.at<uint8_t>(u,v);
	          int32_t rgb = (g << 16) | (g << 8) | g;
	          points.channels[0].values.push_back(*(float*)(&rgb));
	        }
	      }
	    }
	  }
	  else if (encoding == enc::RGB8) {
	    for (int32_t u = 0; u < dense_points.rows; ++u) {
	      for (int32_t v = 0; v < dense_points.cols; ++v) {
        	if (isValidPoint(dense_points(u,v))) {
	          const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
	          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
        	  points.channels[0].values.push_back(*(float*)(&rgb_packed));
	        }
	      }
	    }
	  }
	  else if (encoding == enc::BGR8) {
	    for (int32_t u = 0; u < dense_points.rows; ++u) {
	      for (int32_t v = 0; v < dense_points.cols; ++v) {
	        if (isValidPoint(dense_points(u,v))) {
        	  const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
	          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
        	  points.channels[0].values.push_back(*(float*)(&rgb_packed));
	        }
	      }
	    }
	  }
	  else {
	    ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
	  }

  }
} //namespace stereo_image_proc

