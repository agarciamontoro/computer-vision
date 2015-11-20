#include "image.hpp"

int main(){
    // Image tablero1("imagenes/Tablero1.jpg");
    // Image tablero2("imagenes/Tablero2.jpg");
    //
    // tablero1.setName("Tablero 1.");
    // tablero2.setName("Tablero 2.");
    //
    // vector< pair<Point2f, Point2f> > good_keypoints;
    // vector< pair<Point2f, Point2f> > bad_keypoints;
    //
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(147, 13), Point2f(156, 47)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(504, 95), Point2f(532, 11)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(432, 444), Point2f(527, 466)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(75, 388), Point2f(137,422)));
    //
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(227, 133), Point2f(238, 139)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(396, 169), Point2f(363, 133)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(362, 338), Point2f(413, 337)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(192, 308), Point2f(229, 327)));
    //
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(286,224), Point2f(308,219)));
    // good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(304,251), Point2f(331,245)));
    //
    // Image good_tab2_1 = tablero2.warpPerspective(good_keypoints);
    // good_tab2_1.setName("Resultado bueno de la homografía del tablero 2 en el tablero 1.");
    //
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(148,14), Point2f(156,47)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(174,19), Point2f(177,45)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(198,24), Point2f(198,43)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(223,30), Point2f(221,40)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(141,40), Point2f(155,72)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(168,46), Point2f(176,70)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(191,50), Point2f(197,68)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(217,56), Point2f(219,66)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(136,63), Point2f(153,95)));
    // bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(162,69), Point2f(174,93)));
    //
    // Image bad_tab2_1 = tablero2.warpPerspective(bad_keypoints);
    // bad_tab2_1.setName("Resultado malo de la homografía del tablero 2 en el tablero 1.");
    //
    // tablero1.draw();
    // waitKey(0);
    //
    // tablero2.draw();
    // waitKey(0);
    //
    // good_tab2_1.draw();
    // waitKey(0);
    //
    // bad_tab2_1.draw();
    // waitKey(0);
    //
    // destroyAllWindows();

    Image yosemite1("imagenes/yosemite1.jpg");
    Image yosemite2("imagenes/yosemite2.jpg");

    Image y1_kp = yosemite1.drawDetectedFeatures(Scalar(0,0,255));
    Image y2_kp = yosemite2.drawDetectedFeatures(Scalar(255,0,0));

    y1_kp.setName("Yosemite - 1");
    y2_kp.setName("Yosemite - 2");

    y1_kp.draw();
    y2_kp.draw();

    waitKey(0);
    //destroyAllWindows();

    Image mosaic = yosemite1.createMosaic(yosemite2);
    mosaic.setName("Yosemite - Mosaic");
    mosaic.draw();
    waitKey(0);

    destroyAllWindows();
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
    */
}
