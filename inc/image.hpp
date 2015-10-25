#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <math.h>

using namespace cv;
using namespace std;

class Image{
private:
    Mat image;

    double gaussianFunction(double x, double sigma);
    Mat gaussMask(double sigma);

public:
    Image(string filename);

    void draw();

};
