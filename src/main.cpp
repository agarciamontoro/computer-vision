#include "image.hpp"
#include "camera.hpp"
#include <utility>
#include <iostream>
#include <ctime>

using namespace std;

int main(){
    theRNG().state = clock();
    Camera cam;
    cam.randomFinite(0.0, 1.0);
    cam.printCamera();

    vector<Vec3f> points;

    for (double x1 = 0.1; x1 <= 1.0; x1 += 0.1) {
        for (double x2 = 0.1; x2 <= 1.0; x2 += 0.1) {
            points.push_back(Vec3f(0,x1,x2));
            points.push_back(Vec3f(x2,x1,0));
        }
    }

    vector<pair<Vec3f, Vec2f> > projected_points;

    float max_x, max_y, min_x, min_y;

    max_x = max_y = -9999999;
    min_x = min_y = 9999999;

    vector<Vec3f>::iterator it;
    for (it = points.begin(); it != points.end(); ++it){
        Vec3f point = *it;
        Vec2f projected_point = cam.projectPoint(point);

        if(projected_point[0] > max_x)
            max_x = projected_point[0];
        if(projected_point[0] < min_x)
            min_x = projected_point[0];
        if(projected_point[1] > max_y)
            max_y = projected_point[1];
        if(projected_point[1] < min_y)
            min_y = projected_point[1];

        pair<Vec3f, Vec2f> match(point, projected_point);

        projected_points.push_back(match);
    }

    cout << min_x << ", " << max_x << endl;
    cout << min_y << ", " << max_y << endl;

    Camera estimated(projected_points);

    for (it = points.begin(); it != points.end(); ++it){
        Vec3f point = *it;
        Vec2f projected_point = estimated.projectPoint(point);

        if(projected_point[0] > max_x)
            max_x = projected_point[0];
        if(projected_point[0] < min_x)
            min_x = projected_point[0];
        if(projected_point[1] > max_y)
            max_y = projected_point[1];
        if(projected_point[1] < min_y)
            min_y = projected_point[1];
    }

    cout << min_x << ", " << max_x << endl;
    cout << min_y << ", " << max_y << endl;

    estimated.printCamera();
    cout << "Error:" << estimated.error(cam) << endl;

    Scalar red(0, 0, 255);
    Scalar green(0, 255, 0);

    int img_size = 400;

    Mat canvas(img_size, img_size, CV_8UC3);

    for (it = points.begin(); it != points.end(); ++it){
        Vec3f point = *it;
        Vec2f projected_point = cam.projectPoint(point);
        Vec2f estimated_point = estimated.projectPoint(point);

        Point draw_projected((projected_point[0]-min_x)*img_size/(max_x-min_x), (projected_point[1]-min_y)*img_size/(max_y-min_y));
        Point draw_estimated((estimated_point[0]-min_x)*img_size/(max_x-min_x), (estimated_point[1]-min_y)*img_size/(max_y-min_y));

        circle(canvas, draw_projected, 2, green);
        circle(canvas, draw_estimated, 1, red);
    }

    namedWindow( "Points", WINDOW_AUTOSIZE );
    imshow( "Points", canvas );

    waitKey(0);

}
