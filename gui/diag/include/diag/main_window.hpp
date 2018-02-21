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
        namespace diag
        {
            class MainWindow : public QMainWindow
            {
            public:
                explicit MainWindow(QWidget* parent = nullptr);
            private:
                std::shared_ptr<Ui::MainWindow> m_ui;
            };
        }
    }
}

#endif // MAIN_WINDOW_HPP
