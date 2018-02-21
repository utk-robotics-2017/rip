#include "diag/main_window.hpp"
#include "ui_main_window.h"

namespace rip
{
    namespace gui
    {
        namespace diag
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