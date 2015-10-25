#include "image.hpp"
#include <iostream>


double Image::gaussianFunction(double x, double sigma){
    return exp(-0.5*pow(x/sigma, 2));
}

Mat Image::gaussMask(double sigma){
    //Gaussian function needs to be smapled between -3*sigma and +3*sigma
    int mask_size = floor(3*2*sigma) + 1;

    Mat gauss_mask = Mat(1,mask_size,CV_32FC1);

    // It is necessary to normalize the mask, so the sum of its elements
    // needs to be saved.
    float norm = 0;

    for (int i = 0; i < mask_size; i++) {
        gauss_mask.at<float>(0,i) = gaussianFunction(i-mask_size/2, sigma);

        // Updates the sum of all the mask values.
        norm += gauss_mask.at<float>(0,i);
    }

    // Normalizes the gauss mask.
    gauss_mask = gauss_mask / norm;

    return gauss_mask;
}

Image::Image(string filename){
    this->image = imread(filename);
}

void Image::draw(){
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", this->image );
}
