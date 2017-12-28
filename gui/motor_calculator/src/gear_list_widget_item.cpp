#include "gear_list_widget_item.hpp"
#include "ui_gear_list_widget_item.h"

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {

            GearListWidgetItem::GearListWidgetItem(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::GearListWidgetItem)
            {
                m_ui->setupUi(this);
                connect(m_ui->left, SIGNAL(textChanged(QString)), this, SLOT(updated()));
                connect(m_ui->right, SIGNAL(textChanged(QString)), this, SLOT(updated()));
            }

            GearListWidgetItem::~GearListWidgetItem()
            {
                delete m_ui;
            }

            double GearListWidgetItem::ratio() const
            {
                bool ok;
                double left = m_ui->left->text().toDouble(&ok);
                if(!ok)
                {
                    return 0.0;
                }
                double right = m_ui->right->text().toDouble(&ok);
                if(right == 0.0 || !ok)
                {
                    return 0.0;
                }
                return left / right;
            }

            void GearListWidgetItem::updated()
            {
                emit modified();
            }

        }
    }
}
