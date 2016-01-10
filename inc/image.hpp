#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
    int ID;

    void imageInit(string filename, string name, bool flag_color);

    double gaussianFunction(double x, double sigma);
    Mat getGaussMask(double sigma);
    Mat convolution1D(const Mat& signal_vec, const Mat& mask, enum border_id border_type);
    Mat convolution2D(const Mat& signal_mat, const Mat& mask, enum border_id border_type);
    void copyTo(Mat dst);
    Mat findHomography(vector< pair<Point2f,Point2f> > matches);
    Mat detectFeatures(enum detector_id det_id, vector<KeyPoint> &keypoints);
    pair< vector<Point2f>, vector<Point2f> > match(Image matched, enum descriptor_id descriptor, enum detector_id detector);

public:
    ~Image();

    Image(string filename );
    Image(string filename, string name );
    Image(string filename, bool flag_color);
    Image(string filename, string name, bool flag_color);
    Image(Mat img, string name = "Image");
    Image(const Image& clone);

    const Image operator-(const Image rhs) const;
    const Image operator+(const Image rhs) const;

    int numChannels();
    int rows();
    int cols();
    string getName();
    void setName(string name);

    Image lowPassFilter(double sigma);
    Image highPassFilter(double sigma);
    Image hybrid(Image high_freq, double sigma_low, double sigma_high);

    Image reduceHalf();
    Image pyramidDown(double sigma = 1.0);
    Image makePyramidCanvas(int num_levels);

    Image warpPerspective(vector< pair<Point2f,Point2f> > keypoints);
    Image createMosaic(Image matched);
    Image overlapContours(double low, double high, Scalar color = Scalar(0,0,255));

    void draw();
    void drawMatches(Image other);
    Image drawDetectedFeatures(Scalar color = Scalar(0,0,255), enum descriptor_id descriptor = descriptor_id::BRUTE_FORCE, enum detector_id detector = detector_id::ORB);

    friend Image makeHybridCanvas(Image low, Image high, double sigma_low, double sigma_high);
    friend Image createMosaic_N(vector<Image> &images);

    bool findAndDrawChessBoardCorners(Size pattern_size, vector<Point2f> &corners);

};

#endif
