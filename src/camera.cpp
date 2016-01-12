#include "camera.hpp"
#include <iostream>
#include <ctime>


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
    // // Normalization: translates all the points so that the centroid of the
    // // set of all points is in the origin; then, scales all the points so that
    // // the average distance to the origin is sqrt(2).
    //
    // // Obtains both centroids
    // Vec3f world_centroid(0.0, 0.0, 0.0);
    // Vec2f image_centroid(0.0, 0.0);
    //
    // for (unsigned int i = 0; i < matches.size(); i++) {
    //     Vec3f world_point = matches[i].first;
    //     Vec2f image_point = matches[i].second;
    //
    //     world_centroid += world_point;
    //     image_centroid += image_point;
    // }
    //
    // for (size_t i = 0; i < 3; i++) {
    //     world_centroid[i] /= matches.size();
    // }
    // for (size_t i = 0; i < 2; i++) {
    //     image_centroid[i] /= matches.size();
    // }
    //
    // // Translates the centroid to the origin and
    // // obtains both average distances.
    // float world_distance = 0.0;
    // float image_distance = 0.0;
    //
    // for (unsigned int i = 0; i < matches.size(); i++) {
    //     matches[i].first -= world_centroid;
    //     matches[i].second -= image_centroid;
    //
    //     world_distance += norm(matches[i].first);
    //     image_distance += norm(matches[i].second);
    // }
    //
    // world_distance /= matches.size();
    // image_distance /= matches.size();
    //
    // float world_scale = sqrt(2)/world_distance;
    // float image_scale = sqrt(2)/image_distance;
    //
    // // Scales all the points so that the average distance is sqrt(2)
    // for (unsigned int i = 0; i < matches.size(); i++) {
    //     for (size_t j = 0; j < 2; j++) {
    //         matches[i].first[j] *= world_scale;
    //         matches[i].second[j] *= image_scale;
    //     }
    //     matches[i].first[2] *= world_scale;
    // }
    //
    // // Generation of both matrices
    // Mat world_trans = Mat::eye(4, 4, CV_32F) * world_scale;
    // Mat image_trans = Mat::eye(3, 3, CV_32F) * image_scale;
    //
    // world_trans.at<float>(3,3) = 1;
    // image_trans.at<float>(2,2) = 1;
    //
    // for (size_t i = 0; i < 3; i++) {
    //     world_trans.at<float>(i,3) = world_centroid[i];
    // }
    // for (size_t i = 0; i < 2; i++) {
    //     image_trans.at<float>(i,2) = image_centroid[i];
    // }
    //
    // // Build the equations system.
    Mat mat_system, sing_values, l_sing_vectors, r_sing_vectors;

    for (unsigned int i = 0; i < matches.size(); i++) {
        Vec3f pt_3D = matches[i].first;
        Vec2f pt_2D = matches[i].second;

        float coeffs[2][12] = {
            {0,0,0,0, -pt_3D[0], -pt_3D[1], -pt_3D[2], -1, pt_2D[1]*pt_3D[0], pt_2D[1]*pt_3D[1], pt_2D[1]*pt_3D[2], pt_2D[1]},
            {pt_3D[0], pt_3D[1], pt_3D[2], 1, 0,0,0,0, -pt_2D[0]*pt_3D[0], -pt_2D[0]*pt_3D[1], -pt_2D[0]*pt_3D[2], -pt_2D[0]}
        };

        mat_system.push_back( Mat(2, 12, CV_32F, coeffs) );
    }

    // Solve the equations system using SVD decomposition
    SVD::compute( mat_system, sing_values, l_sing_vectors, r_sing_vectors );

    Mat last_row = r_sing_vectors.row(r_sing_vectors.rows-1);
    //
    // Mat image_trans_inverted;
    // invert(image_trans, image_trans_inverted);
    //
    // this->camera = image_trans_inverted * last_row.reshape(1,3) * world_trans;
    this->camera = last_row.reshape(1,3);
}

/**
 * Generates a random finite (nonsingular) camera with matrix values in the
 * range [min, max]
 * @param min Lower limit of the values range. Defaults to 0.0
 * @param max Upper limit of the values range. Defaults to 1.0
 */
void Camera::randomFinite(float min, float max){
    do{
        theRNG().state = clock();
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

float Camera::error(const Camera& other){
    float norm_this = 0, norm_other = 0;
    bool found = false;

    for (size_t i = 0; i < 3 && !found; i++) {
        float current_this = this->camera.at<float>(0,i);
        float current_other = other.camera.at<float>(0,i);
        if(current_this != 0 && current_other != 0){
            norm_this = current_this;
            norm_other = current_other;
            found = true;
        }
    }

    assert(norm_this != 0 && norm_other != 0);

    Mat cam_1 = this->camera / norm_this;
    Mat cam_2 = other.camera / norm_other;

    return norm(cam_1, cam_2, NORM_L2SQR);
}
