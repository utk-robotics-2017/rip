#include <QApplication>

#include "compute_thread.hpp"
#include "main_window.hpp"

using ComputeThread = rip::gui::pathplanner::ComputeThread;
using MainWindow = rip::gui::pathplanner::MainWindow;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    ComputeThread::getInstance()->start();

    MainWindow mw;
    mw.showMaximized();

    return app.exec();
}
