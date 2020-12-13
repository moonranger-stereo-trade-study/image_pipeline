#include "Halide.h"
#include <stdio.h>
#include "halide_image_io.h"
#include <opencv2/core/mat.hpp>

using namespace Halide::Tools;

int disparity_in_halide(const cv::Mat_<char>& left_rect, const cv::Mat_<char>& right_rect, cv::Mat_<int16_t>& disparity16, int64_t max_d);
