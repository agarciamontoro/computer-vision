#include "image.hpp"

int main(){
    ///////////////////////////////////////// APARTADO 1 /////////////////////////////////////////

    Image tablero1("imagenes/Tablero1.jpg");
    Image tablero2("imagenes/Tablero2.jpg");

    tablero1.setName("Tablero 1.");
    tablero2.setName("Tablero 2.");

    vector< pair<Point2f, Point2f> > good_keypoints;
    vector< pair<Point2f, Point2f> > bad_keypoints;

    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(147, 13), Point2f(156, 47)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(504, 95), Point2f(532, 11)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(432, 444), Point2f(527, 466)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(75, 388), Point2f(137,422)));

    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(227, 133), Point2f(238, 139)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(396, 169), Point2f(363, 133)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(362, 338), Point2f(413, 337)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(192, 308), Point2f(229, 327)));

    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(286,224), Point2f(308,219)));
    good_keypoints.push_back(pair<Point2f, Point2f>(Point2f(304,251), Point2f(331,245)));

    Image good_tab2_1 = tablero2.warpPerspective(good_keypoints);
    good_tab2_1.setName("Resultado bueno de la homografía del tablero 2 en el tablero 1.");

    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(148,14), Point2f(156,47)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(174,19), Point2f(177,45)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(198,24), Point2f(198,43)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(223,30), Point2f(221,40)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(141,40), Point2f(155,72)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(168,46), Point2f(176,70)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(191,50), Point2f(197,68)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(217,56), Point2f(219,66)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(136,63), Point2f(153,95)));
    bad_keypoints.push_back(pair<Point2f, Point2f>(Point2f(162,69), Point2f(174,93)));

    Image bad_tab2_1 = tablero2.warpPerspective(bad_keypoints);
    bad_tab2_1.setName("Resultado malo de la homografía del tablero 2 en el tablero 1.");

    tablero1.draw();
    tablero2.draw();
    good_tab2_1.draw();
    bad_tab2_1.draw();
    waitKey(0);

    destroyAllWindows();

    ///////////////////////////////////////// APARTADO 2 /////////////////////////////////////////

    Image yosemite1("imagenes/yosemite1.jpg");
    Image yosemite2("imagenes/yosemite2.jpg");

    Image y1_kp_ORB = yosemite1.drawDetectedFeatures(Scalar(0,0,255), descriptor_id::BRUTE_FORCE, detector_id::ORB);
    Image y2_kp_ORB = yosemite2.drawDetectedFeatures(Scalar(0,0,255), descriptor_id::BRUTE_FORCE, detector_id::ORB);

    y1_kp_ORB.setName("Yosemite - 1 - ORB - Brute Force");
    y2_kp_ORB.setName("Yosemite - 2 - ORB - Brute Force");

    Image y1_kp_BRISK = yosemite1.drawDetectedFeatures(Scalar(255,0,0), descriptor_id::BRUTE_FORCE, detector_id::BRISK);
    Image y2_kp_BRISK = yosemite2.drawDetectedFeatures(Scalar(255,0,0), descriptor_id::BRUTE_FORCE, detector_id::BRISK);

    y1_kp_BRISK.setName("Yosemite - 1 - BRISK - Brute Force");
    y2_kp_BRISK.setName("Yosemite - 2 - BRISK - Brute Force");

    y1_kp_ORB.draw();
    y2_kp_ORB.draw();

    y1_kp_BRISK.draw();
    y2_kp_BRISK.draw();

    waitKey(0);
    destroyAllWindows();

    ///////////////////////////////////////// APARTADO 3 /////////////////////////////////////////

    yosemite1.drawMatches(yosemite2);
    waitKey(0);
    destroyAllWindows();

    ///////////////////////////////////////// APARTADO 4 /////////////////////////////////////////

    vector<Image> yosemite_images;
    yosemite_images.push_back(yosemite1);
    yosemite_images.push_back(yosemite2);

    Image yosemite_mosaic = createMosaic_N(yosemite_images);
    yosemite_mosaic.setName("Yosemite mosaic");
    yosemite_mosaic.draw();
    waitKey(0);
    destroyAllWindows();

    ///////////////////////////////////////// APARTADO 5 /////////////////////////////////////////

    Image ETSIIT01("imagenes/mosaico002.jpg");
    Image ETSIIT02("imagenes/mosaico003.jpg");
    Image ETSIIT03("imagenes/mosaico004.jpg");
    Image ETSIIT04("imagenes/mosaico005.jpg");
    Image ETSIIT05("imagenes/mosaico006.jpg");
    Image ETSIIT06("imagenes/mosaico007.jpg");
    Image ETSIIT07("imagenes/mosaico008.jpg");
    Image ETSIIT08("imagenes/mosaico009.jpg");
    Image ETSIIT09("imagenes/mosaico010.jpg");
    Image ETSIIT10("imagenes/mosaico011.jpg");

    vector<Image> ETSIIT_images;
    ETSIIT_images.push_back(ETSIIT01);
    ETSIIT_images.push_back(ETSIIT02);
    ETSIIT_images.push_back(ETSIIT03);
    ETSIIT_images.push_back(ETSIIT04);
    ETSIIT_images.push_back(ETSIIT05);
    ETSIIT_images.push_back(ETSIIT06);
    ETSIIT_images.push_back(ETSIIT07);
    ETSIIT_images.push_back(ETSIIT08);
    ETSIIT_images.push_back(ETSIIT09);
    ETSIIT_images.push_back(ETSIIT10);

    Image ETSIIT_mosaic = createMosaic_N(ETSIIT_images);
    ETSIIT_mosaic.setName("ETSIIT mosaic");
    ETSIIT_mosaic.draw();
    waitKey(0);
    destroyAllWindows();
}
