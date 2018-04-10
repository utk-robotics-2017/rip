#ifndef PATH_WIDGET_OUTER_HPP
#define PATH_WIDGET_OUTER_HPP

#include <QWidget>

namespace Ui
{
    class PathWidgetOuter;
}

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class PathWidgetOuter : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit PathWidgetOuter(QWidget* parent = nullptr);
                    ~PathWidgetOuter();

                private slots:
                    void setAnimate();

                private:
                    Ui::PathWidgetOuter* m_ui;
                };
            }
        }
    }
}

#endif // PATH_WIDGET_OUTER_HPP
