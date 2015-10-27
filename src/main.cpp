#include "image.hpp"

int main(){
     Image img("imagenes/einstein.bmp");

     Image convoluted_img = img.convolution2D(5.0);
     convoluted_img.draw();

     waitKey(0);
     destroyAllWindows();
}
