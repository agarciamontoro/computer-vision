#include "image.hpp"

int main(){
     Image fish("imagenes/fish.bmp");
     Image submarine("imagenes/submarine.bmp");

     submarine.hybrid(fish,3.0,4.0).draw();

     Image bike("imagenes/bicycle.bmp");
     Image motorbike("imagenes/motorcycle.bmp",false);

     Image hybrid_canvas = makeHybridCanvas(motorbike,bike,3.5,2.5);

     hybrid_canvas.draw();

     waitKey(0);
     destroyAllWindows();
}
