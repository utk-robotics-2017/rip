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
            /**
             * The main window for this application
             */
            class MainWindow : public QMainWindow
            {
                Q_OBJECT
            public:
                /**
                 * Construcor
                 */
                explicit MainWindow(QWidget* parent = nullptr);

                ~MainWindow();

            private slots:
                /**
                 * Import the robot dimensions
                 */
                void importRobot();

                /**
                 * Export the robot dimensions
                 */
                void exportRobot();

                /**
                 * Import the config for the path planning
                 */
                void importConfig();

                /**
                 * Export the config for the path planning
                 */
                void exportConfig();

                /**
                 * Import the geometry of the obstacles
                 */
                void importObstacles();

                /**
                 * Export the geometry of the obstacles
                 */
                void exportObstacles();

                /**
                 * Export the trajectory
                 */
                void exportTrajectory();

            private:
                std::shared_ptr<Ui::MainWindow> m_ui; //!< shared_ptr as unique_ptr can't handle
                std::shared_ptr<Settings> m_settings;
            };
        }
    }
}

#endif // MAIN_WINDOW_HPP
