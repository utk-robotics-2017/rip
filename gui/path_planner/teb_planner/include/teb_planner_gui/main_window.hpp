#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <memory>
#include <QMainWindow>

#include <teb_planner_gui/settings.hpp>

namespace Ui
{
    class MainWindow;
}

namespace rip
{
    namespace gui
    {
        namespace  tebplanner
        {
            class MainWindow : public QMainWindow
            {
                Q_OBJECT
            public:
                explicit MainWindow(QWidget* parent = nullptr);

                ~MainWindow();

            private:
                std::shared_ptr<Ui::MainWindow> m_ui; //!< shared_ptr as unique_ptr can't handle
                std::shared_ptr<Settings> m_settings;
            };
        }
    }
}

#endif // MAIN_WINDOW_HPP
