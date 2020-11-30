/*********************************************************************
 * Took the original processor.h file and made this for the 
 * last part of processing.
 *********************************************************************/

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "stereo_image_proc/rectification_processing.h"

namespace stereo_image_proc {

  bool RectificationProcessing::rectificationProcessing(const sensor_msgs::ImageConstPtr& raw_image,
                                const image_geometry::PinholeCameraModel& model,
                       image_proc::ImageSet& output, int flags, image_proc::Processor mono_processor)
  {
	  
	  return mono_processor.process(raw_image, model, output, flags);
  }
} //namespace stereo_image_proc

