#include "D2D_Gui.hpp"

using namespace std ; 

int main(int argc, char *argv[]){
    QApplication application(argc, argv);
    D2DMainWindow window(argc, argv) ;
    window.show();
    return application.exec();
}

