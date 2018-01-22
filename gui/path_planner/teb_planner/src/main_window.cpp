#include <teb_planner_gui/main_window.hpp>
#include "ui_main_window.h"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            MainWindow::MainWindow(QWidget* parent)
                : QMainWindow(parent)
                , m_ui(new Ui::MainWindow)
            {
                m_ui->setupUi(this);
            }
        }
    }
}
