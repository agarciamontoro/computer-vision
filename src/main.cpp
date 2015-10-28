#include "image.hpp"

int main(){
     Image bike("imagenes/bicycle.bmp");
     Image motorbike("imagenes/motorcycle.bmp",false);

     Image hybrid_canvas = makeHybridCanvas(motorbike,bike,4.0,3.0);

     hybrid_canvas.setName("Hybrid canvas.");
     hybrid_canvas.draw();

     Image fish("imagenes/fish.bmp");
     Image submarine("imagenes/submarine.bmp");

     Image hybrid = submarine.hybrid(fish,4.0,3.0);
     Image pyramid_canvas = hybrid.makePyramidCanvas(5);

     pyramid_canvas.setName("Gaussian pyramid.");
     pyramid_canvas.draw();

     waitKey(0);
     destroyAllWindows();
}
