#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <memory>

#include <QMainWindow>

#include "preferences_manager.hpp"
#include "settings_manager.hpp"

namespace Ui
{
    class MainWindow;
}

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class MainWindow : public QMainWindow
            {
                Q_OBJECT
            public:
                explicit MainWindow(QWidget* parent = nullptr);
                ~MainWindow();

            public slots:
                void courseUpdated();

            private:
                Ui::MainWindow* m_ui;

                std::shared_ptr<Preferences> m_preferences_manager;
                std::shared_ptr<Settings> m_settings_manager;
            }; // class MainWindow
        } // namespace pathplanner
    } // namespace gui
} // namespace rip

#endif // MAIN_WINDOW_HPP
