#include "Halide.h"
#include <stdio.h>
#include "halide_image_io.h"
#include <opencv2/core/mat.hpp>

using namespace Halide::Tools;

int points2_in_halide(cv::Mat_<cv::Vec3f>& mat);
