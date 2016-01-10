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

    cout << "\tImage::findHomography : sigma_9 = " << sing_values.at<float>(8,0) << endl;

    return last_row.reshape(1,3);
}

Mat Image::detectFeatures(enum detector_id det_id, vector<KeyPoint> &keypoints){
    // Declare detector
    Ptr<Feature2D> detector;

    // Define detector
    if (det_id == detector_id::ORB) {
        // Declare ORB detector
        detector = ORB::create(
            500,                //nfeatures = 500
            1.2f,               //scaleFactor = 1.2f
            4,                  //nlevels = 8
            21,                 //edgeThreshold = 31
            0,                  //firstLevel = 0
            2,                  //WTA_K = 2
            ORB::HARRIS_SCORE,  //scoreType = ORB::HARRIS_SCORE
            21,                 //patchSize = 31
            20                  //fastThreshold = 20
        );
    }
    else{
        // Declare BRISK and BRISK detectors
        detector = BRISK::create(
            55,   // thresh = 30
    		8,    // octaves = 3
    		1.5f  // patternScale = 1.0f
        );
    }

    // Declare array for storing the descriptors
    Mat descriptors;

    // Detect and compute!
    detector->detect(this->image, keypoints);
    detector->compute(this->image,keypoints,descriptors);

    return descriptors;
}

pair< vector<Point2f>, vector<Point2f> > Image::match(Image matched, enum descriptor_id descriptor , enum detector_id detector){
    // 1 - Get keypoints and its descriptors in both images
    vector<KeyPoint> keypoints[2];
    Mat descriptors[2];

    descriptors[0] = this->detectFeatures(detector, keypoints[0]);
    descriptors[1] = matched.detectFeatures(detector, keypoints[1]);

    // 2 - Match both descriptors using required detector
    // Declare the matcher
    Ptr<DescriptorMatcher> matcher;

    // Define the matcher
    if (descriptor == descriptor_id::BRUTE_FORCE) {
        // For ORB and BRISK descriptors, NORM_HAMMING should be used.
        // See http://sl.ugr.es/norm_ORB_BRISK
        matcher = new BFMatcher(NORM_HAMMING, true);
    }
    else{
        matcher = new FlannBasedMatcher();
        // FlannBased Matcher needs CV_32F descriptors
        // See http://sl.ugr.es/FlannBase_32F
        for (size_t i = 0; i < 2; i++) {
            if (descriptors[i].type() != CV_32F) {
                descriptors[i].convertTo(descriptors[i],CV_32F);
            }
        }
    }

    // Match!
    vector<DMatch> matches;
    matcher->match( descriptors[0], descriptors[1], matches );

    // 3 - Create lists of ordered keypoints following obtained matches
    vector<Point2f> ordered_keypoints[2];

    for( unsigned int i = 0; i < matches.size(); i++ )
    {
      // Get the keypoints from the matches
      ordered_keypoints[0].push_back( keypoints[0][matches[i].queryIdx].pt );
      ordered_keypoints[1].push_back( keypoints[1][matches[i].trainIdx].pt );
    }

    return pair< vector<Point2f>, vector<Point2f> >(ordered_keypoints[0], ordered_keypoints[1]);
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

    vector<Point2f> corners(4), corners_trans(4);

    corners[0] = Point2f(0,0);
    corners[1] = Point2f(this->cols(),0);
    corners[2] = Point2f(this->cols(),this->rows());
    corners[3] = Point2f(0,this->rows());

    perspectiveTransform(corners, corners_trans, homography);

    float min_x, min_y, max_x, max_y;
    min_x = min_y = +INF;
    max_x = max_y = -INF;
    for (int j = 0; j < 4; j++) {
        min_x = min(corners_trans[j].x, min_x);
        max_x = max(corners_trans[j].x, max_x);

        min_y = min(corners_trans[j].y, min_y);
        max_y = max(corners_trans[j].y, max_y);
    }

    Mat dst(Size(max_x-min_x,max_y-min_y), this->image.type());

    // Define translation homography
    Mat trans_homography = Mat::eye(3,3,homography.type());
    trans_homography.at<float>(0,2) = -min_x;
    trans_homography.at<float>(1,2) = -min_y;

    cv::warpPerspective( this->image, dst, trans_homography*homography, dst.size() );

    return Image(dst);
}

Image Image::createMosaic(Image matched){
    assert(this->image.type() == matched.image.type());

    // Find homography transforming "matched" image plane into own plane:
    pair< vector<Point2f>, vector<Point2f> > matched_points = this->match(matched, descriptor_id::BRUTE_FORCE, detector_id::ORB);
    Mat homography = cv::findHomography(matched_points.second, matched_points.first, cv::RANSAC, 1);

    // Build the mosaic canvas and declare a header for its left half
    Mat mosaic(Size(this->cols() + matched.cols(), this->rows()), CV_8UC3, Scalar(0,0,0));
    Mat left_slot  = mosaic( Rect(0,0,matched.cols(),matched.rows()) );

    // Copy own image to the mosaic
    this->image.copyTo( left_slot );

    // Copy "matched" image to the mosaic applying the homography
    cv::warpPerspective( matched.image, mosaic, homography, mosaic.size(), INTER_LINEAR, BORDER_TRANSPARENT );

    return Image(mosaic);
}

Image createMosaic_N(vector<Image> &images){
    // Number of images and middle image index
    int N = images.size();
    int mid_idx = N/2; // Integer division, works both with odd and even numbers.

    // Image and homography types
    int img_type = images.front().image.type();
    int homo_type = CV_64F;

    // Declare homographies vectors
    vector<Mat> left_homographies, right_homographies;

    // Homographies between images on the left side of the central one
    for (int i = 0; i < mid_idx; i++) {
        // Compute matched points between image i and i+1
        pair< vector<Point2f>, vector<Point2f> > matched_points = images[i].match(images[i+1], descriptor_id::BRUTE_FORCE, detector_id::ORB);

        // Find homography transforming image i plane into image i+1 plane
        left_homographies.push_back(cv::findHomography(matched_points.first, matched_points.second, cv::RANSAC, 1));
    }

    // Homographies between images on the right side of the central one
    for (int i = N-1; i > mid_idx; i--) {
        // Compute matched points between image i and i-1
        pair< vector<Point2f>, vector<Point2f> > matched_points = images[i].match(images[i-1], descriptor_id::BRUTE_FORCE, detector_id::ORB);

        // Find homography transforming image i plane into image i-1 plane
        right_homographies.push_back(cv::findHomography(matched_points.first, matched_points.second, cv::RANSAC, 1));
    }

    int left_size = left_homographies.size();
    // Compute left homographies composition and store them in the same vector
    for (int i = 0; i < left_size; i++) {
        Mat homo_composition = Mat::eye(3,3,homo_type);
        for (int j = left_size-1; j >= i; j--) {
            homo_composition = homo_composition * left_homographies[j];
        }
        left_homographies[i] = homo_composition;
    }

    int right_size = right_homographies.size();
    // Compute right homographies composition and store them in the same vector
    for (int i = 0; i < right_size; i++) {
        Mat homo_composition = Mat::eye(3,3,right_homographies[0].type());
        for (int j = right_size-1; j >= i; j--) {
            homo_composition = homo_composition * right_homographies[j];
        }
        right_homographies[i] = homo_composition;
    }

    // Declare a vector with all the homographies without translation to the mosaic coordinates
    vector<Mat> homographies;
    reverse(right_homographies.begin(),right_homographies.end());

    homographies.insert(homographies.end(),left_homographies.begin(),left_homographies.end());
    homographies.push_back(Mat::eye(3,3,homo_type));
    homographies.insert(homographies.end(),right_homographies.begin(),right_homographies.end());

    // Get homography image of the corner coordinates from all the images to obtain mosaic size
    vector<Point2f> corners_all(4), corners_all_t(4);
    float min_x, min_y, max_x, max_y;
    min_x = min_y = +INF;
    max_x = max_y = -INF;

    for (int i = 0; i < N; i++) {
        corners_all[0] = Point2f(0,0);
        corners_all[1] = Point2f(images[i].cols(),0);
        corners_all[2] = Point2f(images[i].cols(),images[i].rows());
        corners_all[3] = Point2f(0,images[i].rows());

        perspectiveTransform(corners_all, corners_all_t, homographies[i]);

        for (int j = 0; j < 4; j++) {
            min_x = min(min(corners_all[j].x,corners_all_t[j].x), min_x);
            max_x = max(max(corners_all[j].x,corners_all_t[j].x), max_x);

            min_y = min(min(corners_all[j].y,corners_all_t[j].y), min_y);
            max_y = max(max(corners_all[j].y,corners_all_t[j].y), max_y);
        }
    }
    int mosaic_cols = max_x - min_x;
    int mosaic_rows = max_y - min_y;

    // Create mosaic canvas
    Size mosaic_size(mosaic_cols, mosaic_rows);
    Mat mosaic(mosaic_size, img_type, Scalar(0,0,0));

    // Define translation homography
    Mat trans_homography = Mat::eye(3,3,homo_type);
    trans_homography.at<double>(0,2) = -min_x;
    trans_homography.at<double>(1,2) = -min_y;

    for (size_t i = 0; i < homographies.size(); i++) {
        Mat curr_homography = trans_homography * homographies[i];
        cv::warpPerspective( images[i].image, mosaic, curr_homography, mosaic_size, INTER_LINEAR, BORDER_TRANSPARENT );
    }

    return Image(mosaic);
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

Image Image::drawDetectedFeatures(Scalar color, enum descriptor_id descriptor, enum detector_id detector){
    // Retrieve features
    vector<KeyPoint> keypoints;
    this->detectFeatures(detector,keypoints);

    Mat img_with_kp = this->image.clone();

    // Overlap features
    drawKeypoints(this->image, keypoints, img_with_kp, color);

    return Image(img_with_kp);
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

Image Image::drawMatches(Image other){
    vector<KeyPoint> keypoints[2];
    Mat descriptors[2];

    descriptors[0] = this->detectFeatures(detector_id::ORB, keypoints[0]);
    descriptors[1] = other.detectFeatures(detector_id::ORB, keypoints[1]);

    Ptr<DescriptorMatcher> matcher = new BFMatcher(NORM_HAMMING, true);
    vector<DMatch> matches;
    matcher->match( descriptors[0], descriptors[1], matches );

    Mat draw_matches;
    cv::drawMatches(this->image, keypoints[0], other.image, keypoints[1], matches, draw_matches);

    return Image(draw_matches);
}

bool Image::findAndDrawChessBoardCorners(Size pattern_size, vector<Point2f> &corners){
    int flags = CV_CALIB_CB_ADAPTIVE_THRESH +
                CV_CALIB_CB_NORMALIZE_IMAGE +
                CALIB_CB_FAST_CHECK;
    bool success = findChessboardCorners(this->image, pattern_size,
                                         corners, flags);

    if(success){
        cornerSubPix(this->image, corners, Size(5, 5), Size(-1, -1),
                     TermCriteria());
        drawChessboardCorners(this->image, pattern_size, Mat(corners), true);
    }

    return success;
}

float Image::computeAndDrawEpiLines(Image &other, int num_lines){
    RNG rng;
    theRNG().state = clock();

    pair<vector<Point2f>, vector<Point2f> > matches;
    matches = this->match(other, descriptor_id::BRUTE_FORCE, detector_id::ORB);

    Mat fund_mat;
    vector<Vec3f> lines_1, lines_2;
    vector<unsigned char> mask;
    fund_mat = findFundamentalMat(matches.first, matches.second,
                                  CV_FM_8POINT | CV_FM_RANSAC,
                                  1.,0.99, mask );

    vector<Point2f> good_matches_1;
    vector<Point2f> good_matches_2;

    for (size_t i = 0; i < mask.size(); i++) {
        if(mask[i] == 1){
            good_matches_1.push_back(matches.first[i]);
            good_matches_2.push_back(matches.second[i]);
        }
    }

    computeCorrespondEpilines(good_matches_1, 1, fund_mat, lines_1);
    computeCorrespondEpilines(good_matches_2, 2, fund_mat, lines_2);

    vector<cv::Vec3f>::const_iterator it;

    // Draws both sets of epipolar lines and computes the distances between
    // the lines and their corresponding points.
    float distance_1 = 0.0, distance_2 = 0.0;
    for (size_t i = 0; i < lines_1.size(); i++) {
        Vec2f point_1 = good_matches_1[i];
        Vec2f point_2 = good_matches_2[i];

        Vec3f line_1 = lines_1[i];
        Vec3f line_2 = lines_2[i];

        // Draws only num_lines lines
        if(i % (lines_1.size()/num_lines) == 0 ){
            Scalar color(rng.uniform(0, 255),
                         rng.uniform(0, 255),
                         rng.uniform(0, 255));

            line(this->image,
                 Point(0,
                       -line_1[2]/line_1[1]),
                 Point(this->cols(),
                       -(line_1[2] + line_1[0]*this->cols())/line_1[1]),
                 color
                 );

            line(other.image,
                 Point(0,
                       -line_2[2]/line_2[1]),
                 Point(other.cols(),
                       -(line_2[2] + line_2[0]*other.cols())/line_2[1]),
                 color
                 );
        }

        // Error computation with distance point-to-line
        distance_1 += abs(line_1[0]*point_1[0] +
                          line_1[1]*point_1[1] +
                          line_1[2]) /
                      sqrt(line_1[0]*line_1[0] + line_1[1]*line_1[1]);

        distance_2 += abs(line_2[0]*point_2[0] +
                          line_2[1]*point_2[1] +
                          line_2[2]) /
                      sqrt(line_2[0]*line_2[0] + line_2[1]*line_2[1]);
     }

     return (distance_1+distance_2)/(2*lines_1.size());
}
