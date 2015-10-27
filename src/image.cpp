#include "image.hpp"
#include <iostream>


double Image::gaussianFunction(double x, double sigma){
    return exp(-0.5*pow(x/sigma, 2));
}

Mat Image::getGaussMask(double sigma){
    //Gaussian function needs to be smapled between -3*sigma and +3*sigma
    int mask_size = 2*round(3*sigma) + 1;

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

Mat Image::convolution1D(const Mat& signal_vec, const Mat& mask, enum border_id border_type){
/*
    vector<Mat> signal_channels(num_channels);
    vector<Mat> result_channels(num_channels);

    split(signal_vec, signal_channels);
    // Channel processing
    for (int i = 0; i < num_channels; i++) {
        // Auxiliar matrix with borders specified by border_type
        // We also declare pointer to its centre and to its borders.
        Mat aux_result, aux_result_center, aux_result_left, aux_result_right;
        Rect ROI_centre, ROI_left, ROI_right;

        ROI_centre = Rect(border_size, 0, signal_vec.cols,1);
        ROI_left = Rect(0,0,border_size,1);
        ROI_right = Rect(border_size+signal_vec.cols,0,border_size,1);

        aux_result = Mat(1,signal_vec.cols+mask.cols-1,CV_32FC1);
        aux_result_center = Mat(aux_result, ROI_center);
        aux_result_left = Mat(aux_result, ROI_left);
        aux_result_right = Mat(aux_result, ROI_right);

        // Centre filled with initial data
        aux_result_centre = signal_channels[i].clone();

        // Borders filled with different type depending on the user choice
        if (border_type == BORDER_REFLECT) {
            aux_result_left = Mat(signal_channels[i], Rect(0,0,border_size,1).clone().flip();
            aux_result_right = Mat(signal_channels[i], Rect(signal_vec.cols-border_size,0,border_size,1).clone().flip();
        }
        else{
            aux_result_left = zeros(1,border_size,CV_32FC1);
            aux_result_right = zeros(1,border_size,CV_32FC1);
        }



    }
*/
    assert(signal_vec.rows == 1 && mask.rows == 1 && mask.cols < signal_vec.cols);

    // Number of additional pixels at each side to make the convolution.
    int border_size = mask.cols/2;
    // Number of channels of the image: useful because we have to treat each channel separately
    int num_channels = signal_vec.channels();

    // Initialization of source vector with additional borders.
    Mat bordered;
    copyMakeBorder(signal_vec,bordered,0,0,border_size,border_size,border_type,0.0);

    // Splitting of the bordered vector for making a per-channel processing
    vector<Mat> bordered_channels(num_channels);
    split(bordered, bordered_channels);

    // Declaration of the result vector -with same size and type as the
    // original signal vector- and its splitted channels.
    Mat result = Mat(signal_vec.size(), signal_vec.type());
    vector<Mat> result_channels(num_channels);
    split(result, result_channels);

    // Per-channel processing
    Mat masked_channel, source_channel, result_channel, converted_mask;

    // The mask and the bordered channels need to have the same type.
    // Both of them are converted to CV_32FC1 in order not to lose
    // precision.
    mask.convertTo(converted_mask,CV_32FC1);

    for (int i = 0; i < num_channels; i++) {
        // Channel type conversion
        bordered_channels[i].convertTo(source_channel, CV_32FC1);
        result_channels[i].convertTo(result_channel, CV_32FC1);

        for (int j = 0; j < result.cols; j++) {
            // We focus on the zone centered at j with mask width
            masked_channel = source_channel(Rect(j,0,mask.cols,1));

            // Element-wise multiplication of the focused zone with the mask
            // and summatory of the result.
            // sum() returns a Scalar; i.e., a 1x4 vector, but we need the first one.
            result_channel.at<float>(0,j)  = sum(masked_channel.mul(converted_mask))[0];
        }

        result_channel.convertTo(result_channels[i],result_channels[i].type());
    }

    // Merging again the processed channels
    merge(result_channels, result);

    return result;
}

void Image::convolution2D(double sigma){
    Mat result = Mat(this->image.size(), this->image.type());

    Mat gaussMask = getGaussMask(sigma);

    Mat result_row;
    for (int i = 0; i < result.rows; i++) {
        // We replace the row i for a copy of the convoluted row
        convolution1D(this->image.row(i), gaussMask, REFLECT).copyTo(result.row(i));
    }

    Mat transposed_col;
    for (int j = 0; j < result.cols; j++) {
        // Same as before but transposing the column into a row and viceversa
        transpose(result.col(j),transposed_col);
        transpose(convolution1D(transposed_col, gaussMask, REFLECT), result.col(j));
    }

    this->image = result;
}


Image::Image(string filename){
    this->image = imread(filename);
}

void Image::draw(){
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", this->image );
}
