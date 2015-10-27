#include "image.hpp"

int main(){
     Image bike("imagenes/bicycle.bmp");
     Image motorbike("imagenes/motorcycle.bmp",false);

     motorbike.hybrid(bike,5.0,3.0).draw();

     waitKey(0);
     destroyAllWindows();
}
