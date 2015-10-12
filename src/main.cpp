#include "image.hpp"

int main(){
     Image img("imagenes/cat.bmp");
     img.draw();

     waitKey(0);
     destroyAllWindows();
}
