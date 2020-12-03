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

#ifndef IMAGE_PROC_RECT_W_HALIDE_H
#define IMAGE_PROC_RECT_W_HALIDE_H

namespace image_proc
{
	class RectifyWithHalide
	{
		typedef image_proc::RectifyConfig Config;

		public: 
			static sensor_msgs::ImagePtr rectifyWithHalide(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg, image_geometry::PinholeCameraModel model_, /* boost::recursive_mutex config_mutex_,*/
			     Config config_);

	};
}
#endif
