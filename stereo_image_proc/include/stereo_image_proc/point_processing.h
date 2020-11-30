/*********************************************************************
 * Took the original processor.h file and made this for the 
 * last part of processing.
 *********************************************************************/
#ifndef STEREO_IMAGE_PROC_POINT_PROC_H
#define STEREO_IMAGE_PROC_POINT_PROC_H

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "stereo_image_proc/processor.h"


namespace stereo_image_proc {

class PointProcessing
{
	public:
		static void pointProcessing(const stereo_msgs::DisparityImage& disparity,
        	             const cv::Mat& color, const std::string& encoding,
                	     const image_geometry::StereoCameraModel& model,
	                     sensor_msgs::PointCloud& points, cv::Mat_<cv::Vec3f> dense_points);
};
} //namespace stereo_image_proc

#endif
