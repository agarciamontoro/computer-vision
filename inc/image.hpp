#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <math.h>

#include "consts.hpp"

using namespace cv;
using namespace std;

class Image{
private:
    static int num_images;

    Mat image;
    string name;

    double gaussianFunction(double x, double sigma);
    Mat getGaussMask(double sigma);
    Mat convolution1D(const Mat& signal_vec, const Mat& mask, enum border_id border_type);
    Mat convolution2D(const Mat& signal_mat, const Mat& mask, enum border_id border_type);


public:
    ~Image();
    
    Image(string filename, bool flag_color = true);
    Image(Mat img);

    const Image operator-(const Image rhs) const;
    const Image operator+(const Image rhs) const;

    int numChannels();

    Image lowPassFilter(double sigma);
    Image highPassFilter(double sigma);

    Image hybrid(Image high_freq, double sigma_low, double sigma_high);

    void draw();

};
