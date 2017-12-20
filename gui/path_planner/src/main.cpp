#include <QApplication>

#include "main_window.hpp"

using MainWindow = rip::gui::pathplanner::MainWindow;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    MainWindow mw;
    mw.show();

    return app.exec();
}
