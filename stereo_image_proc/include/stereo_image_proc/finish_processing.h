/*********************************************************************
 * Took the original processor.h file and made this for the 
 * last part of processing.
 *********************************************************************/
#ifndef STEREO_IMAGE_PROC_FINISH_PROC_H
#define STEREO_IMAGE_PROC_FINISH_PROC_H

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "stereo_image_proc/processor.h"
#include "stereo_image_proc/process_disparity.h"


namespace stereo_image_proc {

struct StereoImageSet
{
  image_proc::ImageSet left;
  image_proc::ImageSet right;
  stereo_msgs::DisparityImage disparity;
  sensor_msgs::PointCloud points;
  sensor_msgs::PointCloud2 points2;
};

class FinishProcessing
{
	public:
		static bool finishProcessing(const sensor_msgs::ImageConstPtr& left_raw,
        	       const sensor_msgs::ImageConstPtr& right_raw,
	               const image_geometry::StereoCameraModel& model,
        	       StereoImageSet& output, int flags);
};
} //namespace stereo_image_proc

#endif
