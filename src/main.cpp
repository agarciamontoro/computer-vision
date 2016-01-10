#include "image.hpp"
#include "camera.hpp"
#include <utility>
#include <iostream>

using namespace std;

void updateMinMax(float &min_x, float &max_x, float &min_y, float &max_y, Vec2f projected_point){
    if(projected_point[0] > max_x)
        max_x = projected_point[0];
    if(projected_point[0] < min_x)
        min_x = projected_point[0];
    if(projected_point[1] > max_y)
        max_y = projected_point[1];
    if(projected_point[1] < min_y)
        min_y = projected_point[1];
}

int main(){
    // // Simulated random camera
    // Camera simulated;
    // simulated.randomFinite(0.0, 1.0);
    //
    // // Generation of 3D points
    // vector<Vec3f> points;
    // for (double x1 = 0.1; x1 <= 1.0; x1 += 0.1) {
    //     for (double x2 = 0.1; x2 <= 1.0; x2 += 0.1) {
    //         points.push_back(Vec3f(0,x1,x2));
    //         points.push_back(Vec3f(x2,x1,0));
    //     }
    // }
    //
    // // Projection of points and extraction of minimum and maximum coordinates
    // // (for visual purposes only)
    // vector<pair<Vec3f, Vec2f> > projected_points;
    //
    // float max_x, max_y, min_x, min_y;
    //
    // max_x = max_y = -9999999;
    // min_x = min_y = +9999999;
    //
    // for (size_t i = 0; i < points.size(); i++) {
    //     Vec3f point = points[i];
    //     Vec2f projected_point = simulated.projectPoint(point);
    //
    //     pair<Vec3f, Vec2f> match(point, projected_point);
    //     projected_points.push_back(match);
    //
    //     updateMinMax(min_x, max_x, min_y, max_y, projected_point);
    // }
    //
    // // Generation of an estimated camera with 3D-2D points pairs
    // Camera estimated(projected_points);
    //
    // // Update of the minimum and maximum coordinates with the estimated camera
    // for (size_t i = 0; i < points.size(); i++) {
    //     Vec3f point = point[i];
    //     Vec2f projected_point = estimated.projectPoint(point);
    //
    //     updateMinMax(min_x, max_x, min_y, max_y, projected_point);
    // }
    //
    // // Frobenius norm of the difference of both simulated and estimated cameras
    // cout << "Error:" << estimated.error(simulated) << endl;
    //
    // // Drawing of both set of points in a single image, transforming [min_x, max_x]
    // // and [min_y, max_y] intervals into the [0,img_size] interval.
    // const int img_size = 400;
    // const float scale_x = img_size/(max_x-min_x);
    // const float scale_y = img_size/(max_y-min_y);
    //
    // const Scalar red(0, 0, 255);
    // const Scalar green(0, 255, 0);
    //
    // Mat canvas(img_size, img_size, CV_8UC3);
    //
    // for (size_t i = 0; i < points.size(); i++) {
    //     Vec3f point = points[i];
    //     Vec2f projected_point = simulated.projectPoint(point);
    //     Vec2f estimated_point = estimated.projectPoint(point);
    //
    //     Point draw_projected((projected_point[0]-min_x)*scale_x, (projected_point[1]-min_y)*scale_y);
    //     Point draw_estimated((estimated_point[0]-min_x)*scale_x, (estimated_point[1]-min_y)*scale_y);
    //
    //     circle(canvas, draw_projected, 1, green);
    //     circle(canvas, draw_estimated, 1, red);
    // }
    //
    // namedWindow( "Points", WINDOW_AUTOSIZE );
    // imshow( "Points", canvas );
    //
    // waitKey(0);
    // destroyAllWindows();

    vector<vector<Point3f> > patterns;
    vector<Point3f> pattern;

    vector<vector<Point2f> > boards;
    vector<Point2f> corners;

    for (size_t i = 0; i < 13; i++) {
        for (size_t j = 0; j < 12; j++) {
            pattern.push_back(Point3f(i,j,0));
        }
    }

    string prefix = "./imagenes/Image";
    string suffix = ".tif";
    string filename;
    for (size_t i = 1; i <= 25; i++) {
        filename = prefix + to_string(i) + suffix;
        Image img(filename, false);
        img.setName(filename);

        if(img.findAndDrawChessBoardCorners(Size(13,12), corners)){
            boards.push_back(corners);
            patterns.push_back(pattern);
            //img.draw();
            //waitKey(0);
            //destroyAllWindows();
        }
    }

    Mat camera_matrix, dist_coeffs;
    vector<Mat> rvecs, tvecs;
    double calibration_error = calibrateCamera(patterns, boards, Size(640,480), camera_matrix, dist_coeffs, rvecs, tvecs);
    cout << camera_matrix << endl;
    cout << "Error:" << calibration_error << endl;
}
