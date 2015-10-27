#include "image.hpp"

int main(){
     Image fish("imagenes/fish.bmp");
     Image submarine("imagenes/submarine.bmp");

     submarine.hybrid(fish,3.0,4.0).draw();

     Image bike("imagenes/bicycle.bmp");
     Image motorbike("imagenes/motorcycle.bmp",false);

     motorbike.hybrid(bike,3.5,2.5).draw();

     waitKey(0);
     destroyAllWindows();
}
