#ifndef __CONSTS_HPP__
#define __CONSTS_HPP__

#include <opencv2/core/core.hpp>

#define INF 2147483647.0

enum border_id{
    REFLECT = cv::BORDER_REFLECT,
    ZEROS = cv::BORDER_CONSTANT
};

enum detector_id{
    ORB,
    BRISK
};

enum descriptor_id{
    BRUTE_FORCE,
    FLANN_BASE
};

#endif
