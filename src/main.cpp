#include "image.hpp"
#include "camera.hpp"
#include <utility>
#include <iostream>

using namespace std;

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

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
    //############################################################# EJERCICIO 1
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
    //     Vec3f point = points[i];
    //     Vec2f projected_point = estimated.projectPoint(point);
    //
    //     updateMinMax(min_x, max_x, min_y, max_y, projected_point);
    // }
    //
    // // Frobenius norm of the difference of simulated and estimated cameras
    // cout << "Error:" << estimated.error(simulated) << endl;
    //
    // // Drawing of both set of points in a single image, transforming
    // // [min_x, max_x]  and [min_y, max_y] intervals into the [0,img_size]
    // // interval.
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
    //     Vec2f simulated_point = simulated.projectPoint(point);
    //     Vec2f estimated_point = estimated.projectPoint(point);
    //
    //     Point draw_simulated((simulated_point[0]-min_x)*scale_x, (simulated_point[1]-min_y)*scale_y);
    //     Point draw_estimated((estimated_point[0]-min_x)*scale_x, (estimated_point[1]-min_y)*scale_y);
    //
    //     circle(canvas, draw_simulated, 3, green);
    //     circle(canvas, draw_estimated, 1, red);
    // }
    //
    // namedWindow( "Points", WINDOW_AUTOSIZE );
    // imshow( "Points", canvas );
    //
    // waitKey(0);
    // destroyAllWindows();

    //############################################################# EJERCICIO 2

    // // Chessboard pattern and a vector to store it as many times as the number
    // // of successfully-detected chessboards, for the later calibration.
    // vector<vector<Point3f> > patterns;
    // vector<Point3f> pattern;
    // for (size_t i = 0; i < 12; i++) {
    //     for (size_t j = 0; j < 13; j++) {
    //         pattern.push_back(Point3f(i,j,0));
    //     }
    // }
    //
    // // Vector to store the corners in each iteration and a vector to keep all
    // // the iterations for the later calibration.
    // vector<vector<Point2f> > boards;
    // vector<Point2f> corners;
    //
    // string prefix, suffix, filename;
    // prefix = "./imagenes/Image";
    // suffix = ".tif";
    //
    // for (size_t i = 1; i <= 25; i++) {
    //     //Open i-th image
    //     filename = prefix + to_string(i) + suffix;
    //     Image img(filename, false);
    //
    //     //If the corners are found, treat them, store the detected corners
    //     // and repeat the pattern for the calibration.
    //     if(img.findAndDrawChessBoardCorners(Size(13,12), corners)){
    //         boards.push_back(corners);
    //         patterns.push_back(pattern);
    //
    //         // Show the image
    //         img.setName(filename);
    //         img.draw();
    //         waitKey(0);
    //         destroyAllWindows();
    //     }
    // }
    //
    // Mat camera_matrix, dist_coeffs;
    // vector<Mat> rvecs, tvecs;
    //
    // // Actual calibration of the camera.
    // // The CALIB_RATIONAL_MODEL flag lowers the error from 127 to 103 :(
    // double calibration_error = calibrateCamera(patterns, boards, Size(640,480),
    //                                            camera_matrix, dist_coeffs,
    //                                            rvecs, tvecs,
    //                                            CV_CALIB_RATIONAL_MODEL);
    //
    // cout << camera_matrix << endl;
    // cout << "Error: " << calibration_error << endl;

    //############################################################# EJERCICIO 3

    // Image vmort_1("./imagenes/Vmort1.pgm");
    // Image vmort_2("./imagenes/Vmort1.pgm");
    //
    // vmort_1.setName("Vmort1");
    // vmort_2.setName("Vmort2");
    //
    // float epilines_error = vmort_1.computeAndDrawEpiLines(vmort_2);
    //
    // vmort_1.draw();
    // vmort_2.draw();
    //
    // cout << "Epilines error: " << epilines_error << endl;
    //
    // waitKey(0);
    // destroyAllWindows();

    //############################################################# EJERCICIO 4


    float intrinsic_params[3][3] = {
        {1839.6300000000001091, 0, 1024.2000000000000455},
        {0, 1848.0699999999999363, 686.5180000000000291},
        {0, 0, 1}
    };

    Mat K(3, 3, CV_64F, intrinsic_params);

    Image reconstruction_0("./imagenes/rdimage.000.ppm");
    Image reconstruction_1("./imagenes/rdimage.001.ppm");
    Image reconstruction_2("./imagenes/rdimage.004.ppm");

    Mat F = reconstruction_0.fundamentalMat(reconstruction_1);
    // Mat fund_02 = reconstruction_0.fundamentalMat(reconstruction_2);
    // Mat fund_12 = reconstruction_1.fundamentalMat(reconstruction_2);

    cout << F << endl;

    Mat E = K.t() * F * K;

    double trace_val = trace(E.t() * E)[0];
    E /= trace_val;

    double norm = sqrt(trace_val/2);
    Mat EtE = E.t() * E;

    double T_x = sqrt(1-EtE.at<double>(0,0));
    double T_y = -EtE.at<double>(0,1) / T_x;
    double T_z = -EtE.at<double>(0,2) / T_x;

    Point3d T(T_x, T_y, T_z);

    Mat row_i, w_i;
    Mat R;
    for (size_t i = 0; i < 3; i++) {
        row_i = E.row(i);
        w_i = row_i.cross(Mat(T).t());

        R.push_back( w_i );
    }

    cout << R << endl << T << endl;
}
