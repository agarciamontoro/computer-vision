#include <opencv2/core/core.hpp>

#define INF 2147483647

enum border_id{
    REFLECT = cv::BORDER_REFLECT,
    ZEROS = cv::BORDER_CONSTANT
};

enum detector_id{
    ORB,
    BRISK
};
