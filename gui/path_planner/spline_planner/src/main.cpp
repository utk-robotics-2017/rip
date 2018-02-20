#include <QApplication>

#include <spline_planner_gui/compute_thread.hpp>
#include <spline_planner_gui/main_window.hpp>

using ComputeThread = rip::gui::splineplanner::ComputeThread;
using MainWindow = rip::gui::splineplanner::MainWindow;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    ComputeThread::getInstance()->start();

    MainWindow mw;
    mw.showMaximized();

    return app.exec();
}
