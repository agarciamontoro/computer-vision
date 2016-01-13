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

void printSection(int num){
    cout << endl;
    cout << "#########################################" << endl;
    cout << "#########################################" << endl;
    cout << "############    SECTION " << num << "   #############" << endl;
    cout << "#########################################" << endl;
    cout << "#########################################" << endl;
    cout << endl;
}

int main(){
    //############################################################# EJERCICIO 1
    printSection(1);
    // Simulated random camera
    Camera simulated;
    simulated.randomFinite(0.0, 1.0);

    // Generation of 3D points
    vector<Vec3f> points;
    for (double x1 = 0.1; x1 <= 1.0; x1 += 0.1) {
        for (double x2 = 0.1; x2 <= 1.0; x2 += 0.1) {
            points.push_back(Vec3f(0,x1,x2));
            points.push_back(Vec3f(x2,x1,0));
        }
    }

    // Projection of points and extraction of minimum and maximum coordinates
    // (for visual purposes only)
    vector<pair<Vec3f, Vec2f> > projected_points;

    float max_x, max_y, min_x, min_y;

    max_x = max_y = -9999999;
    min_x = min_y = +9999999;

    for (size_t i = 0; i < points.size(); i++) {
        Vec3f point = points[i];
        Vec2f projected_point = simulated.projectPoint(point);

        pair<Vec3f, Vec2f> match(point, projected_point);
        projected_points.push_back(match);

        updateMinMax(min_x, max_x, min_y, max_y, projected_point);
    }

    // Generation of an estimated camera with 3D-2D points pairs
    Camera estimated(projected_points);

    // Update of the minimum and maximum coordinates with the estimated camera
    // (in order to update the [min-max]_[x-y] values)
    for (size_t i = 0; i < points.size(); i++) {
        Vec3f point = points[i];
        Vec2f projected_point = estimated.projectPoint(point);

        updateMinMax(min_x, max_x, min_y, max_y, projected_point);
    }

    // Frobenius norm of the difference of simulated and estimated cameras
    cout << "SECTION 1:\tFrobenius norm of the cameras difference:" << endl;
    cout << "\t\t" << estimated.error(simulated) << endl;

    // Drawing of both set of points in a single image, transforming
    // [min_x, max_x]  and [min_y, max_y] intervals into the [0,img_size]
    // interval.
    const int img_size = 400;
    const float scale_x = img_size/(max_x-min_x);
    const float scale_y = img_size/(max_y-min_y);

    const Scalar red(0, 0, 255);
    const Scalar green(0, 255, 0);

    Mat canvas(img_size, img_size, CV_8UC3);

    for (size_t i = 0; i < points.size(); i++) {
        Vec3f point = points[i];
        Vec2f simulated_point = simulated.projectPoint(point);
        Vec2f estimated_point = estimated.projectPoint(point);

        Point draw_simulated((simulated_point[0]-min_x)*scale_x, (simulated_point[1]-min_y)*scale_y);
        Point draw_estimated((estimated_point[0]-min_x)*scale_x, (estimated_point[1]-min_y)*scale_y);

        circle(canvas, draw_simulated, 3, green);
        circle(canvas, draw_estimated, 1, red);
    }

    namedWindow( "Points", WINDOW_AUTOSIZE );
    imshow( "Points", canvas );

    waitKey(0);
    destroyAllWindows();

    //############################################################# EJERCICIO 2
    printSection(2);

    // Chessboard pattern and a vector to store it as many times as the number
    // of successfully-detected chessboards, for the later calibration.
    vector<vector<Point3f> > patterns;
    vector<Point3f> pattern;
    for (size_t i = 0; i < 12; i++) {
        for (size_t j = 0; j < 13; j++) {
            pattern.push_back(Point3f(i,j,0));
        }
    }

    // Vector to store the corners in each iteration and a vector to keep all
    // the iterations for the later calibration.
    vector<vector<Point2f> > boards;
    vector<Point2f> corners;

    string prefix, suffix, filename;
    prefix = "./imagenes/Image";
    suffix = ".tif";

    for (size_t i = 1; i <= 25; i++) {
        //Open i-th image
        filename = prefix + to_string(i) + suffix;
        Image img(filename, false);

        //If the corners are found, treat them, store the detected corners
        // and repeat the pattern for the calibration.
        if(img.findAndDrawChessBoardCorners(Size(13,12), corners)){
            boards.push_back(corners);
            patterns.push_back(pattern);

            // Show the image
            img.setName(filename);
            img.draw();
            waitKey(0);
            destroyAllWindows();
        }
    }

    Mat camera_matrix, dist_coeffs;
    vector<Mat> rvecs, tvecs;

    // Actual calibration of the camera.

    // Without optic distortion
    // The 1st flag disables tangential correction; the other three disable
    // radial correction.
    int flags = CV_CALIB_ZERO_TANGENT_DIST |
                CV_CALIB_FIX_K1 |
                CV_CALIB_FIX_K2 |
                CV_CALIB_FIX_K3;
    double calib_error_without = calibrateCamera(patterns, boards,
                                                 Size(640,480),
                                                 camera_matrix, dist_coeffs,
                                                 rvecs, tvecs,
                                                 flags);

    // Only radial distortion
    flags = CV_CALIB_ZERO_TANGENT_DIST;
    double calib_error_radial = calibrateCamera(patterns, boards,
                                          Size(640,480),
                                          camera_matrix, dist_coeffs,
                                          rvecs, tvecs,
                                          flags);

    // Only tangential distortion
    flags = CV_CALIB_FIX_K1 |
            CV_CALIB_FIX_K2 |
            CV_CALIB_FIX_K3;
    double calib_error_tangent = calibrateCamera(patterns, boards,
                                        Size(640,480),
                                        camera_matrix, dist_coeffs,
                                        rvecs, tvecs,
                                        flags);

    // With all optic distortion
    //By default, tangential and radial correction are enabled
    flags = 0;
    double calib_error = calibrateCamera(patterns, boards, Size(640,480),
                                         camera_matrix, dist_coeffs,
                                         rvecs, tvecs,
                                         flags);

    cout << "SECTION 2:\tCalibration errors:" << endl;
    cout << "\t\t\tWithout distortion:\t" << calib_error_without << endl;
    cout << "\t\t\tWith tang. distortion:\t" << calib_error_tangent << endl;
    cout << "\t\t\tWith radial distortion:\t" << calib_error_radial << endl;
    cout << "\t\t\tWith distortion:\t" << calib_error << endl;

    //############################################################# EJERCICIO 3
    printSection(3);

    Image vmort_1("./imagenes/Vmort1.pgm");
    Image vmort_2("./imagenes/Vmort1.pgm");

    vmort_1.setName("Vmort1");
    vmort_2.setName("Vmort2");

    float epilines_error = vmort_1.computeAndDrawEpiLines(vmort_2);

    vmort_1.draw();
    vmort_2.draw();

    cout << "SECTION 3:\tMean error in the epipolar lines:" << endl;
    cout << "\t\t" << epilines_error << endl;

    waitKey(0);
    destroyAllWindows();

    //############################################################# EJERCICIO 4
    printSection(4);

    double intrinsic_params[3][3] = {
        {1839.6300000000001091, 0, 1024.2000000000000455},
        {0, 1848.0699999999999363, 686.5180000000000291},
        {0, 0, 1}
    };

    Mat K(3, 3, CV_64F, intrinsic_params);

    Image reconstruction_0("./imagenes/rdimage.000.ppm");
    Image reconstruction_1("./imagenes/rdimage.001.ppm");
    Image reconstruction_2("./imagenes/rdimage.004.ppm");

    bool success01, success02, success12;
    Mat R01, R02, R12;
    Point3d T01, T02, T12;

    success01 = reconstruction_0.reconstruction(reconstruction_1, K, R01, T01);
    success02 = reconstruction_0.reconstruction(reconstruction_2, K, R02, T02);
    success12 = reconstruction_1.reconstruction(reconstruction_2, K, R12, T12);

    cout << "SECTION 4:\tSuccess of the three reconstructions:" << endl;
    cout << "\t\t" << success01 << ", " << success02 << ", " << success12 << endl;

}
