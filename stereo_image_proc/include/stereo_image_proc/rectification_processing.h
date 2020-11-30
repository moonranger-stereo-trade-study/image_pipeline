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
#include "image_proc/processor.h"   
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
//#include "depth_image_proc/depth_conversions.h"
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>


namespace stereo_image_proc {

class RectificationProcessing
{
	public:
		bool rectificationProcessing(const sensor_msgs::ImageConstPtr& raw_image,
				const image_geometry::PinholeCamera& model,
        	       image_proc::ImageSet& output, int flags, image_proc::Processor mono_processor);
};
} //namespace stereo_image_proc

#endif
