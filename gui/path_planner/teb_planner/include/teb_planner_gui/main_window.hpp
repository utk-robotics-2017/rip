#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <memory>
#include <QMainWindow>

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


            private:
                std::shared_ptr<Ui::MainWindow> m_ui; //!< shared_ptr as unique_ptr can't handle
            };
        }
    }
}

#endif // MAIN_WINDOW_HPP
