/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/assert.h>
#include "stereo_image_proc/processor.h"
#include "stereo_image_proc/process_disparity.h"
#include <sensor_msgs/image_encodings.h>
#include <cmath>
#include <limits>

namespace stereo_image_proc {

//static 
void ProcessDisparity::processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                       const image_geometry::StereoCameraModel& model,
                                       stereo_msgs::DisparityImage& disparity, StereoType current_stereo_algorithm, 
				       cv::Ptr<cv::StereoBM> block_matcher, cv::Ptr<cv::StereoSGBM> sg_block_matcher, cv::Mat_<int16_t> disparity16, const void* sp)
{
  StereoProcessor SP = *((StereoProcessor*)sp);
  // Fixed-point disparity is 16 times the true value: d = d_fp / 16.0 = x_l - x_r.
  static const int DPP = 16; // disparities per pixel
  static const double inv_dpp = 1.0 / DPP;

  // Block matcher produces 16-bit signed (fixed point) disparity image
  if (current_stereo_algorithm == BM)
#if CV_MAJOR_VERSION >= 3
    block_matcher->compute(left_rect, right_rect, disparity16);
  else
    sg_block_matcher->compute(left_rect, right_rect, disparity16);
#else
    block_matcher(left_rect, right_rect, disparity16);
  else
    sg_block_matcher(left_rect, right_rect, disparity16);
#endif

 // std::cout << "HELLO\n" << c << "," << r << "\n";

  // Stereo parameters
  disparity.f = model.right().fx();
  disparity.T = model.baseline();
  
  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disparity.min_disparity = SP.getMinDisparity();
  disparity.max_disparity = SP.getMinDisparity() + SP.getDisparityRange() - 1;
  disparity.delta_d = inv_dpp;

  int half_max = disparity.max_disparity/2 +1;
  //search params
  int c = left_rect.cols;
  int r = left_rect.rows;


  int half_height_to_match = 75; //AKA hhtm 
  //search
  for(int i=0; i<r; i++)
  {
	  for(int j=0; j<c-2;j++) //do not try to match left images' col0
	  {
		  int min_dif = 1000000000; //TODO make it a flag
		  int ind = j; //matching index on the right
		  for(int l = j-half_max; l<j+half_max; l++)
		  {
			  //only search realistic indices
			  if(l<0 || l>c-2)
				  continue;
			  //TODO include min_disparity
			  int temp_dif = 0;
			  //go through the columns of l, l+1, and l+2 for hhtm 
			  for(int h=-half_height_to_match; h<half_height_to_match; h++)
			  {
				  if(h<0 || h>r)
					  continue;
				  int pix_dif_0 = left_rect.at<int>(h, j) - right_rect.at<int>(h, l);
				  int pix_dif_1 =  left_rect.at<int>(h, j+1) - right_rect.at<int>(h, l+1);
				  int pix_dif_2 =  left_rect.at<int>(h, j+2) - right_rect.at<int>(h, l+2);
				  temp_dif += pix_dif_0*pix_dif_0 + pix_dif_1*pix_dif_1 + pix_dif_2*pix_dif_2;
			  }
			  if(temp_dif<min_dif)
			  {
				  min_dif=temp_dif;
				  ind=l;
			  }
			disparity16.at<int>(i, ind) = 16*(j-ind);
		  }
	  }
  }


  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image& dimage = disparity.image;
  dimage.height = disparity16.rows;
  dimage.width = disparity16.cols;
  dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  // We convert from fixed-point to float disparity and also adjust for any x-offset between
  // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
  disparity16.convertTo(dmat, dmat.type(), inv_dpp, -(model.left().cx() - model.right().cx()));
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)


  
}

} //namespace stereo_image_proc
