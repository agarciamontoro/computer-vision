#include "image.hpp"
#include <iostream>

// Static attributes
int Image::num_images = 0;

/*************************** PRIVATE METHODS ***************************/

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

            // Scalar product between the ROI'd source and the mask
            result_channel.at<float>(0,j)  = masked_channel.dot(converted_mask);
        }

        // Backwards conversion: the result should have the same type as the input image
        result_channel.convertTo(result_channels[i],result_channels[i].type());
    }

    // Merging again the processed channels
    merge(result_channels, result);

    return result;
}

Mat Image::convolution2D(const Mat& signal_mat, const Mat& mask, enum border_id border_type){
    Mat result = Mat(signal_mat.size(), signal_mat.type());

    Mat result_row;
    for (int i = 0; i < result.rows; i++) {
        // Row i is replaced with its convolution
        convolution1D(this->image.row(i), mask, border_type).copyTo(result.row(i));
    }

    Mat transposed_col;
    for (int j = 0; j < result.cols; j++) {
        // Column i is replaced with its convolution ---needs transposing, as convolution1D
        // works with row vectors---.
        transpose(result.col(j),transposed_col);
        transpose(convolution1D(transposed_col, mask, border_type), result.col(j));
    }

    // Returns the convoluted image
    return result;
}

/**************************** PUBLIC METHODS ****************************/

/*_---------------* Destructor *_---------------*/
Image::~Image(){
    num_images--;
}

/*---------------* Constructors *---------------*/

Image::Image(string filename, bool flag_color){
    if(!flag_color)
        this->image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    else
        this->image = imread(filename);

    this->name = "Image " + to_string(num_images);
    num_images++;
}

Image::Image(Mat img){
    this->image = img.clone();
    this->name = "Image " + to_string(num_images);
    num_images++;
}

/*---------------* Operators *---------------*/

const Image Image::operator-(const Image rhs) const{
    assert(this->image.size() == rhs.image.size());

    Mat result = this->image - rhs.image;

    return Image(result);
}

const Image Image::operator+(const Image rhs) const{
    assert(this->image.size() == rhs.image.size());

    Mat result = this->image + rhs.image;

    return Image(result);
}

/*---------------* Other methods *---------------*/

int Image::numChannels(){
    return this->image.channels();
}

int Image::rows(){
    return this->image.rows;
}
int Image::cols(){
    return this->image.cols;
}

void Image::copyTo(Mat dst){
    if (this->numChannels() < dst.channels()) {
        Mat aux_3C;
        cvtColor(this->image,aux_3C,CV_GRAY2RGB);
        aux_3C.copyTo(dst);
    }
    else{
        this->image.copyTo(dst);
    }
}

Image Image::lowPassFilter(double sigma){
    Mat gaussMask = getGaussMask(sigma);

    Mat result  = convolution2D(this->image, gaussMask, REFLECT);

    return Image(result);
}

Image Image::highPassFilter(double sigma){
    return *this - this->lowPassFilter(sigma);
}

Image Image::hybrid(Image high_freq, double sigma_low, double sigma_high){
    assert(this->image.size() == high_freq.image.size());

    Mat result;

    Image low_passed = this->lowPassFilter(sigma_low);
    Image high_passed = high_freq.highPassFilter(sigma_high);

    if(low_passed.numChannels() != high_passed.numChannels()){
        int max_channels = max(low_passed.numChannels(), high_passed.numChannels());

        vector<Mat> low_channels(max_channels);
        vector<Mat> high_channels(max_channels);

        split(low_passed.image, low_channels);
        split(high_passed.image, high_channels);

        if (low_passed.numChannels() < max_channels){
            int diff = max_channels - low_passed.numChannels();

            for (int i = diff-1; i < max_channels; i++) {
                low_channels[0].copyTo(low_channels[i]);
            }
        }
        else if (high_passed.numChannels() < max_channels ) {
            int diff = max_channels - high_passed.numChannels();

            for (int i = diff-1; i < max_channels; i++) {
                high_channels[0].copyTo(high_channels[i]);
            }
        }

        vector<Mat> result_channels(max_channels);

        for (int i = 0; i < max_channels; i++) {
            result_channels[i] = low_channels[i] + high_channels[i];
        }

        merge(result_channels, result);
    }
    else{
        result = low_passed.image + high_passed.image;
    }

    return Image(result);
}

Image Image::reduceHalf(){
    Mat dst_rows = Mat(this->rows()/2, this->cols(),this->image.type());
    Mat dst = Mat(this->rows()/2, this->cols()/2,this->image.type());

    // First remove the odd rows
    for (int i = 0; i < dst_rows.rows; i++) {
        this->image.row(2*i).copyTo(dst_rows.row(i));
    }
    
    // Then, remove the odd columns from the previous output
    for (int i = 0; i < dst.cols; i++) {
        dst_rows.col(2*i).copyTo(dst.col(i));
    }

    return Image(dst);
}

void Image::draw(){
    namedWindow( this->name, WINDOW_AUTOSIZE );
    imshow( this->name, this->image );
}

Image makeHybridCanvas(Image low, Image high, double sigma_low, double sigma_high){
    assert(low.image.size() == high.image.size());

    Image low_passed = low.lowPassFilter(sigma_low);
    Image high_passed = high.highPassFilter(sigma_high);
    Image hybrid = low.hybrid(high,sigma_low,sigma_high);

    Mat canvas = Mat(hybrid.rows(),3*hybrid.cols(),hybrid.image.type());

    vector<Mat> slots(3);

    for (int i = 0; i < 3; i++) {
        slots[i] = canvas( Rect(i*hybrid.cols(),0,hybrid.cols(),hybrid.rows()) );
    }

    low_passed.copyTo(slots[0]);
    high_passed.copyTo(slots[1]);
    hybrid.copyTo(slots[2]);

    return Image(canvas);
}
