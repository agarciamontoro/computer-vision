#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

using namespace cv;
using namespace std;

class Camera{
private:
    Mat camera;

    bool isFinite();

public:

    Camera();
    // Camera(Mat camera);
    // Camera(const Camera& clone);

    void randomFinite(float min = 0.0, float max = 1.0);
    Vec2f projectPoint(Vec3f point);

    void printCamera();
};

#endif
