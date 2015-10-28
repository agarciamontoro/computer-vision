#include "image.hpp"

int main(){
    Image bird("imagenes/bird.bmp");
    Image blurred_bird = bird.lowPassFilter(3.0);

    bird.setName("Bird image.");
    blurred_bird.setName("Low-pass filtered bird image.");

    bird.draw();
    blurred_bird.draw();

    waitKey(0);
    destroyAllWindows();

    Image bike("imagenes/bicycle.bmp");
    Image motorbike("imagenes/motorcycle.bmp",false);

    Image hybrid_canvas = makeHybridCanvas(motorbike,bike,4.0,3.0);

    hybrid_canvas.setName("Hybrid canvas.");
    hybrid_canvas.draw();

    waitKey(0);
    destroyAllWindows();

    Image fish("imagenes/fish.bmp");
    Image submarine("imagenes/submarine.bmp");

    Image hybrid = submarine.hybrid(fish,4.0,3.0);
    Image pyramid_canvas = hybrid.makePyramidCanvas(7);

    pyramid_canvas.setName("Gaussian pyramid.");
    pyramid_canvas.draw();

    waitKey(0);
    destroyAllWindows();

    Image plane("imagenes/plane.bmp");

    Image contours_plane = plane.overlapContours(30,80);

    contours_plane.setName("Detecting contours with Canny filter");
    contours_plane.draw();

    waitKey(0);
    destroyAllWindows();
}
