#include "image.hpp"

int main(){
    Image tablero1("imagenes/Tablero1.jpg");
    Image tablero2("imagenes/Tablero2.jpg");

    tablero1.setName("Tablero 1.");
    tablero2.setName("Tablero 2.");

    vector< pair<Point2f, Point2f> > keypoints;

    keypoints.push_back(pair<Point2f, Point2f>(Point2f(147, 13), Point2f(156, 47)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(504, 95), Point2f(532, 11)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(432, 444), Point2f(527, 466)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(75, 388), Point2f(137,422)));

    keypoints.push_back(pair<Point2f, Point2f>(Point2f(227, 133), Point2f(238, 139)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(396, 169), Point2f(363, 133)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(362, 338), Point2f(413, 337)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(192, 308), Point2f(229, 327)));

    keypoints.push_back(pair<Point2f, Point2f>(Point2f(286,224), Point2f(308,219)));
    keypoints.push_back(pair<Point2f, Point2f>(Point2f(304,251), Point2f(331,245)));

    Image tab2_1 = tablero2.warpPerspective(keypoints);
    tab2_1.setName("Resultado de la homografía del tablero 2 en el tablero 1.");

    tablero1.draw();
    tablero2.draw();
    tab2_1.draw();

    waitKey(0);
    /*
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
    */
}
