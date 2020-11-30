/*********************************************************************
 * Took the original processor.h file and made this for the 
 * last part of processing.
 *********************************************************************/

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "stereo_image_proc/processor.h"

namespace stereo_image_proc {

  static bool FinishProcessing::finishProcessing(const sensor_msgs::ImageConstPtr& left_raw,
               const sensor_msgs::ImageConstPtr& right_raw,
               const image_geometry::StereoCameraModel& model,
               StereoImageSet& output, int flags) const
  {
	// Do block matching to produce the disparity image
	  if (flags & DISPARITY) {
		  StereoProcessor::processDisparity(output.left.rect, output.right.rect, model, output.disparity);
	  }

	  // Project disparity image to 3d point cloud
	  if (flags & POINT_CLOUD) {
		  StereoProcessor::processPoints(output.disparity, output.left.rect_color, output.left.color_encoding, model, output.points);
	  }

	  // Project disparity image to 3d point cloud
	  if (flags & POINT_CLOUD2) {
	    StereoProcessor::processPoints2(output.disparity, output.left.rect_color, output.left.color_encoding, model, output.points2);
	  }

	  return true;
  }
} //namespace stereo_image_proc

