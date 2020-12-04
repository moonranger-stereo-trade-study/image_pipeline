#include "Halide.h"
#include <stdio.h>
#include "halide_image_io.h"

using namespace Halide::Tools;

int main(int argc, char **argv) {
	Halide::Buffer<float> input = load_and_convert_image("rgb.png");
	
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
	convert_and_save_image(output, "blurred.png");
	printf("Success \n");
	return 0;
}	
