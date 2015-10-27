#include "image.hpp"

int main(){
     Image Bike("imagenes/bicycle.bmp");
     Image Motorbike("imagenes/motorcycle.bmp",false);

     Bike.lowPassFilter(2.0).draw();
     Motorbike.highPassFilter(2.0).draw();

     waitKey(0);
     destroyAllWindows();
}
