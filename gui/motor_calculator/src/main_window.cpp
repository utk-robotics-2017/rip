#include "main_window.hpp"
#include "ui_main_window.h"

#include "settings.hpp"

#include <QDir>
#include <QStandardPaths>

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            MainWindow::MainWindow(QWidget* parent)
                : QMainWindow(parent)
                , m_ui(new Ui::MainWindow)
                , m_settings(Settings::getInstance())
            {
                m_ui->setupUi(this);

                QDir app_data(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
                if(!app_data.exists())
                {
                    app_data.mkpath(".");
                    app_data.mkpath("./motors");
                }

                m_settings->load();

                m_ui->motors->setOptions(m_settings->motorNames());
            }

            MainWindow::~MainWindow()
            {
                m_settings->save();
                delete m_ui;
            }
        }
    }
}
