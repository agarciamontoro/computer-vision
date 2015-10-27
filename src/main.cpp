#include "image.hpp"

int main(){
     Image img("imagenes/einstein.bmp");
     Image high_freq = img.highFreq(2.3);

     high_freq.draw();

     waitKey(0);
     destroyAllWindows();
}
