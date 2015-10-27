#include "image.hpp"

int main(){
     Image img("imagenes/cat.bmp");

     img.convolution2D(5.0);
     img.draw();

     waitKey(0);
     destroyAllWindows();
}
