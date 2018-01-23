#ifndef GEAR_LIST_WIDGET_ITEM_HPP
#define GEAR_LIST_WIDGET_ITEM_HPP

#include <QWidget>

namespace Ui
{
    class GearListWidgetItem;
}

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            class GearListWidgetItem : public QWidget
            {
                Q_OBJECT
            public:
                explicit GearListWidgetItem(QWidget* parent = nullptr);

                ~GearListWidgetItem();

                double ratio() const;
            signals:
                void modified();

            private slots:
                void updated();
            private:
                Ui::GearListWidgetItem* m_ui;
            };
        }
    }
}

#endif // GEAR_LIST_WIDGET_ITEM_HPP
