#ifndef POINT_LIST_WIDGET_ITEM_HPP
#define POINT_LIST_WIDGET_ITEM_HPP

#include <QListWidgetItem>

namespace Ui
{
    class PointListWidgetItem;
}


namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class PointListWidgetItem : public QWidget
            {
                Q_OBJECT
            public:
                explicit PointListWidgetItem(QWidget* parent = nullptr);

                ~PointListWidgetItem();

                double x() const;
                void setX(double x);

                double y() const;
                void setY(double y);

            signals:
                void modified();

            public slots:
                void updatedText();

            private:
                Ui::PointListWidgetItem* m_ui;
            };
        }
    }
}

#endif // POINT_LIST_WIDGET_ITEM_HPP
