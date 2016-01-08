#include "image.hpp"
#include "camera.hpp"
#include <utility>
#include <iostream>

using namespace std;

int main(){
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

    vector<Vec3f>::iterator it;
    for (it = points.begin(); it != points.end(); ++it){
        Vec3f point = *it;
        Vec2f projected_point = cam.projectPoint(point);

        pair<Vec3f, Vec2f> match(point, projected_point);

        projected_points.push_back(match);
    }

    Camera estimated(projected_points);
    estimated.printCamera();

    cout << "Error:" << estimated.error(cam) << endl;
}
