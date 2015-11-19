#include "image.hpp"
#include <iostream>

// Static attributes
int Image::num_images = 0;

/*************************** PRIVATE METHODS ***************************/

/**
 * Samples the 1D gaussian function at point x with parameter sigma
 */
double Image::gaussianFunction(double x, double sigma){
    return exp(-0.5*(x*x)/(sigma*sigma));
}

/**
 * Builds a gaussian mask given the parameter sigma. The gaussian function is
 * sampled in the interval [-3*sigma, 3*sigma].
 */
Mat Image::getGaussMask(double sigma){
    // Gaussian function needs to be sampled between -3*sigma and +3*sigma
    // and the mask has to have an odd dimension.
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

/**
 * Returns the result of convolving the uni-dimensional signal_vec with the
 * mask, applying one of two types of borders: REFLECT or ZEROS.
 */
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

    // Per-channel processing: we need the source channels, the masked channels;
    // i.e., the source channel focused in a ROI of the same size as the mask
    // and the result channels.
    Mat source_channel, masked_channel, result_channel;

    for (int i = 0; i < num_channels; i++) {
        // Channel type conversion
        bordered_channels[i].convertTo(source_channel, CV_32FC1);
        result_channels[i].convertTo(result_channel, CV_32FC1);

        // Actual processing
        for (int j = 0; j < result.cols; j++) {
            // We focus on the zone centered at j+mask.cols/2 with mask width
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

/**
 * Returns the result of convolving the two-dimensional signal_vec with a
 * uni-dimensional mask, applied in both rows and columns with one of two types
 * of borders: REFLECT or ZEROS.
 */
Mat Image::convolution2D(const Mat& signal_mat, const Mat& mask, enum border_id border_type){
    Mat result = Mat(signal_mat.size(), signal_mat.type());

    // Row-processing: the uni-dimensional mask is applied to each row separately
    Mat result_row;
    for (int i = 0; i < result.rows; i++) {
        // Row i is replaced with its convolution
        convolution1D(this->image.row(i), mask, border_type).copyTo(result.row(i));
    }

    // Column-processing: the same uni-dimensional mask is applied to each column
    // separately
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

/**
 * Wrapper of cv::copyTo() that can convert one-channel images into three-channels
 * images.
 */
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

void Image::imageInit(string filename, string name, bool flag_color){
    this->image = flag_color ? imread(filename) : imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    this->name = name;

    num_images++;
    this->ID = num_images;
}

Mat Image::findHomography(vector< pair<Point2f,Point2f> > matches){
    // Build the equations system. See http://sl.ugr.es/homography_estimation
    Mat mat_system, sing_values, l_sing_vectors, r_sing_vectors;

    for (unsigned int i = 0; i < matches.size(); i++) {
        Point2f first = matches[i].first;
        Point2f second = matches[i].second;

        float coeffs[2][9] = {
            { -first.x, -first.y, -1., 0., 0., 0., second.x*first.x, second.x*first.y, second.x },
            { 0., 0., 0., -first.x, -first.y, -1., second.y*first.x, second.y*first.y, second.y }
        };

        mat_system.push_back( Mat(2, 9, CV_32FC1, coeffs) );
    }

    // Solve the equations system using SVD decomposition
    SVD::compute( mat_system, sing_values, l_sing_vectors, r_sing_vectors, 0 );

    Mat last_row = r_sing_vectors.row(r_sing_vectors.rows-1);

    return last_row.reshape(1,3);
}

/**************************** PUBLIC METHODS ****************************/

/*_---------------* Destructor *_---------------*/
Image::~Image(){
    num_images--;
}

/*---------------* Constructors *---------------*/

Image::Image(string filename){
    imageInit(filename, "Image", true);
}

Image::Image(string filename, string name){
    imageInit(filename, name, true);
}

Image::Image(string filename, bool flag_color){
    imageInit(filename, "Image", flag_color);
}

Image::Image(string filename, string name, bool flag_color){
    imageInit(filename, name, flag_color);
}

Image::Image(Mat img, string name){
    this->image = img.clone();
    this->name = name;
    num_images++;
    this->ID = num_images;
}

Image::Image(const Image& clone){
    clone.image.copyTo(this->image);
    this->name = clone.name;
    num_images++;
    this->ID = num_images;

}

/*---------------* Operators *---------------*/

/**
 * Wrapper of cv::Mat::operator-()
 */
const Image Image::operator-(const Image rhs) const{
    assert(this->image.size() == rhs.image.size());

    Mat result = this->image - rhs.image;

    return Image(result);
}

/**
 * Wrapper of cv::Mat::operator+()
 */
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

string Image::getName(){
    return this->name;
}

void Image::setName(string name){
    this->name = name;
}

/*
 * Returns the result of applying a 2D convolution with a gaussian mask that is
 * built with the parameter sigma.
 */
Image Image::lowPassFilter(double sigma){
    Mat gaussMask = getGaussMask(sigma);

    Mat result  = convolution2D(this->image, gaussMask, REFLECT);

    return Image(result);
}

/*
 * Returns the result of subtracting the low-pass-filtered source to the source
 * itself, remaining the high frequencies.
 */
Image Image::highPassFilter(double sigma){
    return *this - this->lowPassFilter(sigma);
}

/*
 * Mixes a low-pass-filtered version of the source with a high-pass-filtered
 * version of high_freq image, returning an hybrid image whose appeareance
 * changes dependening on the distance at which the image is seen.
 */
Image Image::hybrid(Image high_freq, double sigma_low, double sigma_high){
    assert(this->image.size() == high_freq.image.size());

    Mat result;

    // Applies low-pass filter to the source and high-pass filter to high_freq.
    Image low_passed = this->lowPassFilter(sigma_low);
    Image high_passed = high_freq.highPassFilter(sigma_high);

    // If the number of channels of both images is different, the image with the
    // minimum number of channels (tested with 1) is expanded to an image with the
    // maximum number of channels (tested with 3) copying the first channel.
    if(low_passed.numChannels() != high_passed.numChannels()){
        int max_channels = max(low_passed.numChannels(), high_passed.numChannels());

        //Both images are splitted in a vector with size = maximum number of channels
        vector<Mat> low_channels(max_channels);
        vector<Mat> high_channels(max_channels);

        split(low_passed.image, low_channels);
        split(high_passed.image, high_channels);

        // The image with less channels is expanded
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

        // Actual processing
        for (int i = 0; i < max_channels; i++) {
            result_channels[i] = low_channels[i] + high_channels[i];
        }

        merge(result_channels, result);
    }
    else{
        // Actual processing if the number of channels is the same.
        result = low_passed.image + high_passed.image;
    }

    return Image(result);
}

/**
 * Downsamples an image reducing its size by half. The returned image is
 * a copy of the source with odd rows and columns removed.
 */
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

/*
 * Returns the next level in a Gaussian pyramid: it simply blurres the source
 * and then downsamples it by half
 */
Image Image::pyramidDown(double sigma){
    return this->lowPassFilter(sigma).reduceHalf();
}

/*
 * Returns an image filled with a Gaussian pyramid. The number of levels of the
 * pyramid is set by num_levels
 */
Image Image::makePyramidCanvas(int num_levels){
    // The size of the canvas is:
    //  width:  image.width + image.width/2
    //  height: image.height
    Mat canvas = Mat::zeros(this->rows(),round(this->cols()*1.5),this->image.type());

    // First pyramid level: the source.
    Image pyramid_level = *this;

    // Places the source in the first half of the canvas
    Mat level_zero = canvas( Rect(0,0,this->cols(),this->rows()) );
    pyramid_level.copyTo(level_zero);


    // The next levels of the pyramid are treated in the loop.
    Mat level_i; //ROI where the pyramid level i will be placed

    // ROI variables
    int left, top, width, height;
    left = this->cols(); // All levels (>0) are placed in the second half of the canvas
    top = 0;
    height = 0;

    for (int i = 1; i < num_levels; i++) {
        // Blurred and downsampled image
        pyramid_level = pyramid_level.pyramidDown();

        top    += height; // The top of the ROI is placed just below the previous level
        width  = pyramid_level.cols();
        height = pyramid_level.rows();

        // ROI
        level_i = canvas( Rect(left,top,width,height) );

        // Actual placing
        pyramid_level.copyTo(level_i);
    }

    return Image(canvas);
}

Image Image::warpPerspective(vector< pair<Point2f,Point2f> > keypoints){
    Mat homography = this->findHomography(keypoints);

    Mat dst(this->image.size(), this->image.type());

    cv::warpPerspective( this->image, dst, homography, this->image.size() );

    return Image(dst);
}

Image Image::overlapContours(double low, double high, Scalar color){
    Mat canny_output;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Blurs the source to get more accurate contours
    Image source = this->lowPassFilter(1.0);

    // Converts source to gray scale
    Mat gray_source;
    if (this->numChannels() > 1) {
        cvtColor(source.image, gray_source, CV_RGB2GRAY);
    }
    else{
        gray_source = source.image;
    }

    // Detects edges using Canny Filter
    Canny( gray_source, canny_output, low, high );

    // Finds contours and store the result in the contours and hierarchy variables.
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

    // Draws red contours
    Image result(*this);

    for( unsigned int i = 0; i< contours.size(); i++ ){
        drawContours( result.image, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    return result;
}

void Image::draw(){
    namedWindow( this->name, WINDOW_AUTOSIZE );

    // Converts to 8-bits unsigned int to avoid problems
    // in OpenCV implementations in Microsoft Windows.
    Mat image_8U;
    this->image.convertTo(image_8U, CV_8U);

    imshow( this->name, image_8U );
}

void Image::drawDetectedFeatures(){
    using namespace cv::detail;
    // Detect features
    // Declare ORB detector and features container
    OrbFeaturesFinder orb;
    ImageFeatures features;

    orb(this->image, features);

    Point2f point;
    Vec3b color(0,0,255);

    for (size_t i = 0; i < features.keypoints.size(); i++) {
        point = features.keypoints[i].pt;
        this->image.at<Vec3b>(point) = color;
    }

    this->draw();
}

/**
 * Returns an image object with the low frequencies image, the high frequencies image and the hybrid image
 * all placed in the same canvas.
 */
Image makeHybridCanvas(Image low, Image high, double sigma_low, double sigma_high){
    assert(low.image.size() == high.image.size());

    // Generates the low frequencies, high frequencies and hybrid images.
    Image low_passed = low.lowPassFilter(sigma_low);
    Image high_passed = high.highPassFilter(sigma_high);
    Image hybrid = low.hybrid(high,sigma_low,sigma_high);

    // Declare the final canvas
    Mat canvas = Mat(hybrid.rows(),3*hybrid.cols(),hybrid.image.type());

    // Obtain the three ROIs needed to place the images in the canvas
    vector<Mat> slots(3);
    for (int i = 0; i < 3; i++) {
        slots[i] = canvas( Rect(i*hybrid.cols(),0,hybrid.cols(),hybrid.rows()) );
    }

    // Places the images in the canvas ROIs
    low_passed.copyTo(slots[0]);
    high_passed.copyTo(slots[1]);
    hybrid.copyTo(slots[2]);

    return Image(canvas);
}
