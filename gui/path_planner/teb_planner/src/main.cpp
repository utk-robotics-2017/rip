#include <QApplication>

#include <teb_planner_gui/compute_thread.hpp>
#include <teb_planner_gui/main_window.hpp>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            int main(int argc, char** argv)
            {
                QApplication app(argc, argv);

                ComputeThread::getInstance()->start();

                MainWindow mw;
                mw.showMaximized();

                return app.exec();
            }
        }
    }
}

int main(int argc, char** argv)
{
    return rip::gui::tebplanner::main(argc, argv);
}
