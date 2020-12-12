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
#include <ros/time.h>

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

  disparity16 = cv::Mat::zeros(left_rect.rows, left_rect.cols, CV_16S);
  cv::Mat_<char> left_rec = cv::Mat::zeros(left_rect.rows, left_rect.cols, CV_8U);
  cv::Mat_<char> right_rec = cv::Mat::zeros(left_rect.rows, left_rect.cols, CV_8U);
  // Block matcher produces 16-bit signed (fixed point) disparity image
  /*if (current_stereo_algorithm == BM)
#if CV_MAJOR_VERSION >= 3
    block_matcher->compute(left_rect, right_rect, disparity16);
  else
    sg_block_matcher->compute(left_rect, right_rect, disparity16);
#else
    block_matcher(left_rect, right_rect, disparity16);
  else
    sg_block_matcher(left_rect, right_rect, disparity16);
#endif*/ 

  // Disparity search range
  disparity.min_disparity = SP.getMinDisparity();
  disparity.max_disparity = SP.getMinDisparity() + SP.getDisparityRange() - 1;
  disparity.delta_d = inv_dpp;
  int max_d = 60; //the actual max disparity value used in the algorithm
  //We do not include min_disparity because min_disparity is 0

  //search params
  int c = disparity16.cols;
  int r = disparity16.rows;
  int part = 2;
  int stride = 1;
  int half_height_to_match = 3; //AKA hhtm 
  int half_width_to_match = 3;

  //std::cout << "HI\n";
  int64_t int_mx = 0x7FFFFFFFFFFFFFFF;
  int64_t min_dif = int_mx; //for one pixel in the left image, what is the difference
  						//between its surrounding pixels and the surrounding 
						//pixels around the closest match in the right image
  ros::Time start = ros::Time::now();
  //Go from top to bottom in the image
  for(int i=0; i < r; i++)
  {
	  for(int j=60; j < c; j+=stride) //go from left to right. Start at column ~60 in LEFT image because the columns <~60 will not
		  				//match left images' cols that will not match with anything in the right image
						//TODO have to enable parameterization for generalization across camera types/setups
	  {
		  
		  min_dif = int_mx; // minimum difference, pixel-wise between a box around left_rect(i,j) 
		  			// and boxes around potential matching indices in the right image
					// The box is (i*half_height_to_match+1) by (2*half_width_to_match+1) (the one corresponding to min_diff)
					// This is actually storing a sum of pixel-wise squared differences
		  int ind = j+1; //index of pixel matchng left_rect(i,j) in the right image
			
		  //search for a match along the same horizontal line in the right image, going at most max_d left and right from the pixel (i,j)
		  for(int rii = j-max_d; rii<j+max_d; rii++) //search up to max_d (disparity) (rii for right image index)
		  {
			  //only search realistic indices
			  if(rii<0 || rii>c)
				  continue;
			  
			  int64_t temp_dif = 0; //store the total sum of squared differences for the square around (i,l)
			  //go though the box around right_image(i, rii) from top to bottom
			  for(int h=i-half_height_to_match; h<i+half_height_to_match; h++)
			  {
				  //only use realistic indices
				  if(h<0 || h>r)
					  continue;
				  //add up squared differences right_image(i, rii) from left to right
				  for(int k=rii-half_width_to_match; k<rii+half_width_to_match; k++)
				  {
					//only use realistic indices
					if(k<0 || k>c && k-rii+j>0 && k-rii+j < c)
						continue;
					//find differences betweent the matching pixels in the box around (i,j) and (i, rii)
				  	int64_t pix_dif_0 = left_rect.at<char>(h, k-rii+j) - right_rect.at<char>(h, k);
					temp_dif += pix_dif_0*pix_dif_0; //add to total sum of squared differences
				  }
				  if(temp_dif>min_dif)
					  break;
			  }
			  //check if this box could have the minimum difference 
			  if(temp_dif<min_dif)
			  {
				  //if so, set the appropriate values
				  min_dif=temp_dif;
				  ind=rii;
			  }
		}
		//calculate the actual disparity. We multiply by 16 because that is what stere_image_proc does
		//RECALL: "Fixed-point disparity is 16 times the true value: d = d_fp / 16.0 = x_l - x_r." from towards the top of this method
		int16_t dispy = 16*(int16_t)(j-ind);
		disparity16.at<int16_t>(i, j) = dispy; //set the (i,j) pixel in the disparity image to be the calcaulated disparity
	  }
  }
  ros::Time end = ros::Time::now();
  std::cout << "Time spent: " << end.toSec() - start.toSec() << "\n";

  // Other Stereo parameters
  disparity.f = model.right().fx();
  disparity.T = model.baseline();

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
