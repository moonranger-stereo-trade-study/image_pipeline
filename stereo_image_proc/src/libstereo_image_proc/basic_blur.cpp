#include "Halide.h"
#include <stdio.h>
#include "halide_image_io.h"
#include "stereo_image_proc/basic_blur.h"
#include <string>

using namespace Halide::Tools;

int basic_blur(int i) {

	printf("From basic_blur.cpp \n");
	Halide::Buffer<float> input = load_and_convert_image("/home/mrak/15618/project/catkin_ws/devel/lib/stereo_image_proc/rgb.png");
	
	Halide::Var x, y, c;
	Halide::Func blurx, blury;
	//Halide::Expr check;

	//if(check = (is_inf(input(0, 0, c))))// == 0)
	//		printf("YAY \n");
	blurx(x, y, c) = ((input(x - 1, y, c) + input(x, y, c) + input(x + 1, y, c)))/3.0f;
	blury(x, y, c) = ((blurx(x, y - 1, c) + blurx(x, y, c) + blurx(x, y + 1, c)))/3.0f;


	Halide::Buffer<float> output(input.width() - 2, input.height() - 2, input.channels());
	output.set_min(1, 1);
	blury.realize(output);
	//std::string s ="blurred";
	//s = s + std::to_string(i) + ".png";
	convert_and_save_image(output, "/home/mrak/15618/project/catkin_ws/devel/lib/stereo_image_proc/blurred2.png");
	printf("Success \n");
	return 0;
}
int main(int argv, char **argc) {
	return 0;
}
