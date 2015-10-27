#include "image.hpp"
#include <iostream>


double Image::gaussianFunction(double x, double sigma){
    return exp(-0.5*(x*x)/(sigma*sigma));
}

Mat Image::getGaussMask(double sigma){
    // Gaussian function needs to be sampled between -3*sigma and +3*sigma
    // and the mask has to have an odd dimension
    int mask_size = 2*round(3*sigma) + 1;

    Mat gauss_mask = Mat(1,mask_size,CV_32FC1);

    // It is necessary to normalize the mask, so the sum of its elements
    // needs to be saved.
    float values_sum = 0;

    // Fills the mask with a sampled gaussian function and saves the sum of all elements
    for (int i = 0; i < mask_size; i++) {
        gauss_mask.at<float>(0,i) = gaussianFunction(i-mask_size/2, sigma);
        values_sum += gauss_mask.at<float>(0,i);
    }

    // Normalizes the gauss mask.
    gauss_mask = gauss_mask / values_sum;

    return gauss_mask;
}

Mat Image::convolution1D(const Mat& signal_vec, const Mat& mask, enum border_id border_type){
    assert(signal_vec.rows == 1 && mask.rows == 1 && mask.cols < signal_vec.cols);

    int num_channels = signal_vec.channels();

    // Initialization of source vector with additional borders.
    int border_size = mask.cols/2;  // Number of pixels added to each side

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

    // The mask and the source/result channels need to have the same type.
    // They are all converted to CV_32FC1 in order not to lose precision.
    Mat converted_mask;
    mask.convertTo(converted_mask,CV_32FC1);

    // Per-channel processing
    Mat source_channel, masked_channel, result_channel;

    for (int i = 0; i < num_channels; i++) {
        // Channel type conversion
        bordered_channels[i].convertTo(source_channel, CV_32FC1);
        result_channels[i].convertTo(result_channel, CV_32FC1);

        // Actual processing
        for (int j = 0; j < result.cols; j++) {
            // We focus on the zone centered at j with mask width
            masked_channel = source_channel(Rect(j,0,mask.cols,1));

            // Element-wise multiplication of the focused zone with the mask
            // and summatory of the result.
            // sum() returns a Scalar; i.e., a 1x4 vector, but we need the first one.
            result_channel.at<float>(0,j)  = sum(masked_channel.mul(converted_mask))[0];
        }

        // Backwards conversion: the result should have the same type as the input image
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
        // Row i is replaced with its convolution
        convolution1D(this->image.row(i), gaussMask, REFLECT).copyTo(result.row(i));
    }

    Mat transposed_col;
    for (int j = 0; j < result.cols; j++) {
        // Column i is replaced with its convolution ---needs transposing, as convolution1D
        // works with row vectors---.
        transpose(result.col(j),transposed_col);
        transpose(convolution1D(transposed_col, gaussMask, REFLECT), result.col(j));
    }

    // Updates the object
    this->image = result;
}


Image::Image(string filename){
    this->image = imread(filename);
}

void Image::draw(){
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", this->image );
}
