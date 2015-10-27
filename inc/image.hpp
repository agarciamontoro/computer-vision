#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <math.h>

#include "consts.hpp"

using namespace cv;
using namespace std;

class Image{
private:
    Mat image;

    double gaussianFunction(double x, double sigma);
    Mat getGaussMask(double sigma);
    Mat convolution1D(const Mat& signal_vec, const Mat& mask, enum border_id border_type);
    Mat convolution2D(const Mat& signal_vec, double sigma);


public:
    Image(string filename);

    void draw();

};
