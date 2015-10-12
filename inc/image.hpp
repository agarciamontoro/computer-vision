#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

using namespace cv;
using namespace std;

class Image{
private:
    Mat image;

public:
    Image(string filename);

    void draw();

};
