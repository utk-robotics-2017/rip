#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <memory>

#include <QMainWindow>

#include "settings.hpp"

namespace Ui
{
    class MainWindow;
}

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            class MainWindow : public QMainWindow
            {
                Q_OBJECT
            public:
                explicit MainWindow(QWidget* parent = nullptr);
                ~MainWindow();
            private:
                Ui::MainWindow* m_ui;
                std::shared_ptr<Settings> m_settings;
            };
        }
    }
}

#endif // MAIN_WINDOW_HPP
