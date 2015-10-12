#include "image.hpp"

Image::Image(string filename){
    this->image = imread(filename);
}

void Image::draw(){
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", this->image );
}
