#ifndef WORLD_WIDGET_OUTER_HPP
#define WORLD_WIDGET_OUTER_HPP

#include <memory>

#include <QWidget>

namespace Ui
{
    class WorldWidgetOuter;
}


namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class WorldWidgetOuter : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit WorldWidgetOuter(QWidget* parent = nullptr);
                    ~WorldWidgetOuter();

                public slots:
                    void worldOptionsChanged();

                private slots:
                    void add();
                    void remove();

                private:
                    Ui::WorldWidgetOuter* m_ui;
                };
            }
        }
    }
}
#endif // WORLD_WIDGET_OUTER_HPP
