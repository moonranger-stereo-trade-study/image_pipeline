#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>


#ifndef STEREO_IMAGE_PROC_POINTS2_NODL_PROC_H
#define STEREO_IMAGE_PROC_POINTS2_NODL_PROC_H


namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class Points2NodeletProcessing
{
	public:
		 static PointCloud2Ptr points2Processing(const cv::Mat_<float> dmat,
                                 cv::Mat_<cv::Vec3f> points_mat,
                                 image_geometry::StereoCameraModel model,
				 const DisparityImageConstPtr& disp_msg, const ImageConstPtr& l_image_msg);
};
} //namespace stereo_image_proc

#endif
