#include "Halide.h"
#include <stdio.h>
#include "halide_image_io.h"
#include <opencv2/core/mat.hpp>
#include "stereo_image_proc/disparity_halide.h"

using namespace Halide::Tools;

int disparity_in_halide(const cv::Mat_<char>& left_rect, const cv::Mat_<char>& right_rect, cv::Mat_<int16_t>& disparity16, int64_t max_d)
{

	//wrap the images in Halide buffers
	Halide::Buffer<uint8_t> left(left_rect.data, left_rect.cols, left_rect.rows, left_rect.channels());
	Halide::Buffer<uint8_t> right(right_rect.data, right_rect.cols, right_rect.rows, right_rect.channels());

	//Variables that will be used in the Funcs below
	Halide::Var x, y, shift;

	//Expressions to clamp values to being in-bounds 
	Halide::Expr clamp_col, crm3, crm2, crm1, crp1, crp2, crp3, ccm3, ccm2, ccm1, ccp1, ccp2, ccp3;
	clamp_col = Halide::max(Halide::min((Halide::cast<int32_t>)(x+shift),right_rect.cols-1),0);

	//expresions to clamp offset values: ccm# means clamp column minus # and ccp# means clamp col plus #.
	//				// crm# means clamp row minus # and crp# means clamp row plus #.
	crm3 = Halide::max(Halide::min((y-3),left_rect.rows-1),0); 
	crm2 = Halide::max(Halide::min((y-2),left_rect.rows-1),0);
	crm1 = Halide::max(Halide::min((y-1),left_rect.rows-1),0);
	crp1 = Halide::max(Halide::min((y+1),left_rect.rows-1),0); 
	crp2 = Halide::max(Halide::min((y+2),left_rect.rows-1),0);
	crp3 = Halide::max(Halide::min((y+3),left_rect.rows-1),0);
	
	ccm3 = Halide::max(Halide::min((x-3),left_rect.cols-1),0);
        ccm2 = Halide::max(Halide::min((x-2),left_rect.cols-1),0);
        ccm1 = Halide::max(Halide::min((x-1),left_rect.cols-1),0);
        ccp1 = Halide::max(Halide::min((x+1),left_rect.cols-1),0);
        ccp2 = Halide::max(Halide::min((x+2),left_rect.cols-1),0);
        ccp3 = Halide::max(Halide::min((x+3),left_rect.cols-1),0);
	
	//Given an x, y, and a shift, return right(x+shift, y);
	Halide::Func shift32("shift32");
	shift32(x, y, shift) = Halide::cast<int32_t>(right(clamp_col, y, 0));
	
	//find the difference between the pixel in left image at (x,y) and pixel in right image at (x+shift, y)
	Halide::Func dif_img32("dif_img32");
	dif_img32(x,y, shift) = (shift32(x,y,shift))-(Halide::cast<int32_t>(left(x,y,0)));

	//square the differences	
	Halide::Func square_img("square_img");
	square_img(x,y,shift) = dif_img32(x,y,shift)*dif_img32(x,y,shift);

	//sum the differences in a 5 x 5 box around (x,y)
	Halide::Func calc_ssd("calc_ssd"); //Hard code the square as being 5 x 5.
	calc_ssd(x,y,shift) = /*square_img(ccm3,crm3,shift) +square_img(ccm2,crm3,shift) +square_img(ccm1,crm3,shift) +square_img(x,crm3,shift) +square_img(ccp1,crm3,shift) +square_img(ccp2,crm3,shift) +square_img(ccp3,crm3,shift) +*/
			/*square_img(ccm3,crm2,shift) +*/square_img(ccm2,crm2,shift) +square_img(ccm1,crm2,shift) +square_img(x,crm2,shift) +square_img(ccp1,crm2,shift) +square_img(ccp2,crm2,shift) +//square_img(ccp3,crm2,shift) +
			/*square_img(ccm3,crm1,shift) +*/square_img(ccm2,crm1,shift) +square_img(ccm1,crm1,shift) +square_img(x,crm1,shift) +square_img(ccp1,crm1,shift) +square_img(ccp2,crm1,shift) +//square_img(ccp3,crm1,shift) +
			/*square_img(ccm3,y,shift) +*/square_img(ccm2,y,shift) +square_img(ccm1,y,shift) +square_img(x,y,shift) +square_img(ccp1,y,shift) +square_img(ccp2,y,shift) +//square_img(ccp3,y,shift) +
			/*square_img(ccm3,crp1,shift) +*/square_img(ccm2,crp1,shift) +square_img(ccm1,crp1,shift) +square_img(x,crp1,shift) +square_img(ccp1,crp1,shift) +square_img(ccp2,crp1,shift) +//square_img(ccp3,crm1,shift) +
			/*square_img(ccm3,crp2,shift) +*/square_img(ccm2,crp2,shift) +square_img(ccm1,crp2,shift) +square_img(x,crp2,shift) +square_img(ccp1,crp2,shift) +square_img(ccp2,crp2,shift); //square_img(ccp3,crm2,shift);// +
			/*square_img(ccm3,crp3,shift) +square_img(ccm2,crp3,shift) +square_img(ccm1,crp3,shift) +square_img(x,crp3,shift) +square_img(ccp1,crp3,shift) +square_img(ccp2,crp3,shift) +square_img(ccp3,crm3,shift);*/


	//Help from: https://halide-lang.org/docs/tutorial_2lesson_13_tuples_8cpp-example.html#a16
	
	//Looking at shifts from -60 to 59
	Halide::RDom r(-60, 59); 

	//Create the expression to evaluate to get a sum of squared differences given x, y 
	Halide::Expr matches = (calc_ssd(x, y, r));
	//find the shift that has the lowest sum of squared differences
	Halide::Tuple best_match = Halide::argmin(matches);
		 
	//we are only concerned with the shift
	Halide::Func best_match_buffer("best_match_buffer");
	best_match_buffer(x,y) = Halide::cast<int16_t>(best_match[0]);
	
	//buffer with all of the shifts
	Halide::Buffer<int16_t> disp=best_match_buffer.realize(disparity16.cols, disparity16.rows);
	
	//write values to the CV mat of disparity16
	for(int i=0; i<disparity16.rows; i++)
	{
		for(int j=60; j<disparity16.cols; j++) //Start at 60 to match sequential algorithm
		{
			disparity16.at<int16_t>(i,j) = -16*(disp(j,i)); //times 16 because that is what stereo_image_proc expects. -16 because otherwise the shifts are all negative
			//std::cout << disp(j,i) << " ";
		}
		//std::cout << "\n";
	}
	
	
	
	return 0;
}
