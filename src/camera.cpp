#include "camera.hpp"
#include <iostream>



/*-------------* Private functions *------------*/

/**
 * Returns wether the current camera is finite; i.e., the 3x3 left submatrix is
 * nonsingular.
 * @return True if the camera is finite; false in any other case.
 */
bool Camera::isFinite(){
    Mat sub = this->camera(Rect(0,0,3,3));
    return determinant(sub) != 0.0;
}

/*-------------* Public functions *-------------*/

/**
 * Dummy constructor that initializes the camera matrix with zero values
 */
Camera::Camera(){
    this->camera = Mat::zeros(3, 4, CV_32F);
}

/**
 * Constructs a camera that fits the 3D <-> 2D matches given
 */
Camera::Camera( vector< pair<Vec3f, Vec2f> > matches ){
    Vec3f world_centroid(0.0, 0.0, 0.0);
    Vec2f image_centroid(0.0, 0.0);

    vector< pair<Vec3f, Vec2f> >::iterator it;
    for (it = matches.begin(); it != matches.end(); ++it){
        pair<Vec3f, Vec2f> match = *it;
        Vec3f world_point = match.first;
        Vec2f image_point = match.second;

        world_centroid += world_point;
        image_centroid += image_point;
    }

    for (size_t i = 0; i < 3; i++) {
        world_centroid[i] /= matches.size();
    }
    for (size_t i = 0; i < 2; i++) {
        image_centroid[i] /= matches.size();
    }
}

/**
 * Generates a random finite (nonsingular) camera with matrix values in the
 * range [min, max]
 * @param min Lower limit of the values range. Defaults to 0.0
 * @param max Upper limit of the values range. Defaults to 1.0
 */
void Camera::randomFinite(float min, float max){
    do{
        randu(this->camera, Scalar::all(min), Scalar::all(max));
    }while( !this->isFinite() );
}

/**
 * Projects the given 3D point with the current camera, obtaining 2D pixel
 * coordinates.
 * @param  point 3D world point, codified as a Vec3f object
 * @return       Returns the camera projection in pixel coordinates, codified
 *               as a Vec2f, of the given 3D point.
 */
Vec2f Camera::projectPoint(Vec3f point){
    float hom_vector[4] = {point[0], point[1], point[2], 1.0};
    Mat hom_point = Mat(4, 1, CV_32F, hom_vector);

    Mat projection = this->camera * hom_point;

    float p_x = projection.at<float>(0)/projection.at<float>(2);
    float p_y = projection.at<float>(1)/projection.at<float>(2);

    return Vec2f(p_x, p_y);
}

/**
 * Prints the matrix to the standard output.
 */
void Camera::printCamera(){
    cout << this->camera << endl;
}
