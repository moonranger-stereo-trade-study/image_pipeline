#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/RectifyConfig.h>
#include "image_proc/rectify_with_halide.h"

namespace image_proc
{
	typedef image_proc::RectifyConfig Config;

	sensor_msgs::ImagePtr RectifyWithHalide::rectifyWithHalide(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg, image_geometry::PinholeCameraModel model_, /* boost::recursive_mutex config_mutex_,*/
			     Config config_)
	{
		//ROS_ERROR("IN RECTIFY WITH HALIDE")
		model_.fromCameraInfo(info_msg);

		  // Create cv::Mat views onto both buffers
		  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
		  cv::Mat rect;

		  // Rectify and publish
		  int interpolation;
		  {
		    // boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
		    interpolation = config_.interpolation;
		  }
		  model_.rectifyImage(image, rect, interpolation);

		  // Allocate new rectified image message
		  sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
		  return rect_msg;
	}

} //namespace image_proc
