#include <QApplication>
#include "main_window.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    rip::gui::motorcalc::MainWindow main_window;
    main_window.showMaximized();

    return app.exec();
}
