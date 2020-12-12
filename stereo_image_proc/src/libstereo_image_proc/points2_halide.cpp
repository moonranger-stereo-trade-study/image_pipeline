#include "stereo_image_proc/points2_halide.h"

const float MISSING_Z = 10000;

int points2_in_halide(cv::Mat_<cv::Vec3f>& mat) {
	
	//printf("In points2_halide \n");
	Halide::Var x, y, z;
	Halide::Buffer<uint8_t> input(mat.data, mat.cols, mat.rows, mat.channels());	
	Halide::Expr c1, c2;
	Halide::Expr value = input(x, y, z);
	Halide::Expr check = Halide::cast<float>(input(0, 0, z));
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	c1 = Halide::operator!=(check, MISSING_Z);
	c2 = Halide::operator~(Halide::is_inf(check));
	c2 = Halide::operator&&(c1, c2);
	value = Halide::cast<float>(value);
	
	Halide::Func check_valid_point;
	check_valid_point(x, y, z) = Halide::cast<uint8_t>(select(c2, value, bad_point));

	check_valid_point.realize(input);
	
	//printf("Success \n");
	return 0;
}
int main(int argv, char **argc) {
	return 0;
}
