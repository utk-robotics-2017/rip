#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>

namespace Ui
{
    class MainWindow;
}

namespace rip
{
    namespace gui
    {
        namespace arduino_gen
        {
            class MainWindow : public QMainWindow
            {
                Q_OBJECT
            public:
                explicit MainWindow(QWidget* widget = nullptr);
                ~MainWindow();
            private:
                Ui::MainWindow* m_ui;
            };
        }
    }
}


#endif //MAIN_WINDOW_HPP