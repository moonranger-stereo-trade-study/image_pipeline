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

	/*halide_dimension_t shape[3] = {{0, disparity16.cols, disparity16.step1(1)}, 
		                       {0, disparity16.rows, disparity16.step1(0)},
				       {0, disparity16.channels(), 1}};
	Halide::Runtime::Buffer<int16_t> disp(disparity16.data, 1, shape);	*/

	//std::cout << "PP1\n";
	//
	Halide::Var x, y, shift;

	Halide::Expr clamp_col, crm3, crm2, crm1, crp1, crp2, crp3, ccm3, ccm2, ccm1, ccp1, ccp2, ccp3;
	clamp_col = Halide::max(Halide::min((Halide::cast<int32_t>)(x+shift),left_rect.cols-1),0);

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
	
	/*Halide::Func blurxr, bluryr, blurxl, bluryl;
        blurxr(x, y) = ((right(ccm1, y, 0) + right(x, y, 0) + right(ccp1, y, 0)))/3.0f;
        bluryr(x, y) = ((blurxr(x, crm1) + blurxr(x, y) + blurxr(x, crp1)))/3.0f;
        blurxl(x, y) = ((left(ccm1, y, 0) + left(x, y, 0) + left(ccp1, y, 0)))/3.0f;
        bluryl(x, y) = ((blurxl(x, crm1) + blurxl(x, y) + blurxl(x, crp1)))/3.0f;*/


	Halide::Func shift32("shift32");
	shift32(x, y, shift) = Halide::cast<int32_t>(right(clamp_col, y, 0));


	//std::cout << "PP3\n";
	
	Halide::Func dif_img32("dif_img32");
	dif_img32(x,y, shift) = (shift32(x,y,shift))-(Halide::cast<int32_t>(left(x,y,0)));

	//std::cout << "PP4\n";
	
	Halide::Func square_img("square_img");
	square_img(x,y,shift) = dif_img32(x,y,shift)*dif_img32(x,y,shift);

	//std::cout << "PP5\n";

	Halide::Func calc_ssd("calc_ssd"); //Hard code the square as being 7 x 7.
	calc_ssd(x,y,shift) = /*square_img(ccm3,crm3,shift) +square_img(ccm2,crm3,shift) +square_img(ccm1,crm3,shift) +square_img(x,crm3,shift) +square_img(ccp1,crm3,shift) +square_img(ccp2,crm3,shift) +square_img(ccp3,crm3,shift) +*/
			/*square_img(ccm3,crm2,shift) +*/square_img(ccm2,crm2,shift) +square_img(ccm1,crm2,shift) +square_img(x,crm2,shift) +square_img(ccp1,crm2,shift) +square_img(ccp2,crm2,shift) +//square_img(ccp3,crm2,shift) +
			/*square_img(ccm3,crm1,shift) +*/square_img(ccm2,crm1,shift) +square_img(ccm1,crm1,shift) +square_img(x,crm1,shift) +square_img(ccp1,crm1,shift) +square_img(ccp2,crm1,shift) +//square_img(ccp3,crm1,shift) +
			/*square_img(ccm3,y,shift) +*/square_img(ccm2,y,shift) +square_img(ccm1,y,shift) +square_img(x,y,shift) +square_img(ccp1,y,shift) +square_img(ccp2,y,shift) +//square_img(ccp3,y,shift) +
			/*square_img(ccm3,crp1,shift) +*/square_img(ccm2,crp1,shift) +square_img(ccm1,crp1,shift) +square_img(x,crp1,shift) +square_img(ccp1,crp1,shift) +square_img(ccp2,crp1,shift) +//square_img(ccp3,crm1,shift) +
			/*square_img(ccm3,crp2,shift) +*/square_img(ccm2,crp2,shift) +square_img(ccm1,crp2,shift) +square_img(x,crp2,shift) +square_img(ccp1,crp2,shift) +square_img(ccp2,crp2,shift); //square_img(ccp3,crm2,shift);// +
			/*square_img(ccm3,crp3,shift) +square_img(ccm2,crp3,shift) +square_img(ccm1,crp3,shift) +square_img(x,crp3,shift) +square_img(ccp1,crp3,shift) +square_img(ccp2,crp3,shift) +square_img(ccp3,crm3,shift);*/



	/*calc_ssd(x,y,shift) = square_img(ccm3,crm3,shift) + square_img(x-2,y-3,shift) + square_img(x-1,y-3,shift) + square_img(x,y-3,shift) + square_img(x+1,y-3,shift) + square_img(x+2,y-3,shift) + square_img(x+3,y-3,shift) +
		  	square_img(x-3,y-2,shift) + square_img(x-2,y-2,shift) + square_img(x-1,y-2,shift) + square_img(x,y-2,shift) + square_img(x+1,y-2,shift) + square_img(x+2,y-2,shift) + square_img(x+3,y-2,shift) +
			square_img(x-3,y-1,shift) + square_img(x-2,y-1,shift) + square_img(x-1,y-1,shift) + square_img(x,y-1,shift) + square_img(x+1,y-1,shift) + square_img(x+2,y-1,shift) + square_img(x+3,y-1,shift) +
			square_img(x-3,y,shift) + square_img(x-2,y,shift) + square_img(x-1,y,shift) + square_img(x,y,shift) + square_img(x+1,y,shift) + square_img(x+2,y,shift) + square_img(x+3,y,shift) +
			square_img(x-3,y+1,shift) + square_img(x-2,y+1,shift) + square_img(x-1,y+1,shift) + square_img(x,y+1,shift) + square_img(x+1,y+1,shift) + square_img(x+2,y+1,shift) + square_img(x+3,y+1,shift) +
			square_img(x-3,y+2,shift) + square_img(x-2,y+2,shift) + square_img(x-1,y+2,shift) + square_img(x,y+2,shift) + square_img(x+1,y+2,shift) + square_img(x+2,y+2,shift) + square_img(x+3,y+2,shift) +
			square_img(x-3,y+3,shift) + square_img(x-2,y+3,shift) + square_img(x-1,y+3,shift) + square_img(x,y+3,shift) + square_img(x+1,y+3,shift) + square_img(x+2,y+3,shift) + square_img(x+3,y+3,shift);*/

	//Help from: https://halide-lang.org/docs/tutorial_2lesson_13_tuples_8cpp-example.html#a16
	
	//std::cout << "PP6\n";

	Halide::RDom r(-60, 59); //TODO if it does not compile, hard code this

	Halide::Expr matches = (calc_ssd(x+2, y+2, r));// < 0x7FFFFFFF;
	Halide::Tuple best_match = Halide::argmin(matches);
		 
	Halide::Func best_match_buffer("best_match_buffer");
	best_match_buffer(x,y) = Halide::cast<int16_t>(best_match[0]);
	
	/*best_match_buffer.realize(disp);*/
	Halide::Buffer<int16_t> disp = best_match_buffer.realize(disparity16.cols-4, disparity16.rows-4,1);
	
	//std::cout << "PP7";

	for(int i=0; i<disparity16.rows-4; i++)
	{
		for(int j=60; j<disparity16.cols-4; j++) //Start at 60 to match sequential algorithm
		{
			disparity16.at<int16_t>(i+2,j+2) = -16*(disp(j,i));
			//std::cout << disp(j,i) << " ";
		}
		//std::cout << "\n";
	}
	
	
	
	return 0;
}
