#ifndef MAIN_WINDOW_HPP_HPP
#define MAIN_WINDOW_HPP_HPP

#include <QMainWindow>

namespace Ui {
    class MainWindow;
}


namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class MainWindow : public QMainWindow
                {
                    Q_OBJECT
                public:
                    explicit MainWindow(QWidget* parent=nullptr);
                    ~MainWindow();

                private slots:
                    void importWorld();
                    void exportWorld();
                    void importWaypoints();
                    void exportWaypoints();
                    void importRobot();
                    void exportRobot();
                private:
                    Ui::MainWindow* m_ui;
                };
            }
        }
    }
}

#endif //MAIN_WINDOW_HPP_HPP
