#include "preferences_widget.hpp"
#include "ui_preferences_widget.h"

#include <QFileDialog>

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            PreferencesWidget::PreferencesWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::PreferencesWidget)
                , m_preferences(Preferences::getInstance())
            {
                m_ui->setupUi(this);

                m_ui->distance_unit_combobox->addItems(QStringList() << "in" << "mm" << "cm");
                m_ui->distance_unit_combobox->setCurrentText(QString::fromStdString(m_preferences->getDistanceUnitText()));
                connect(m_ui->distance_unit_combobox, SIGNAL(currentIndexChanged(QString)), m_preferences.get(), SLOT(setDistanceUnit(QString)));

                m_ui->time_unit_combobox->addItems(QStringList() << "s" << "ms");
                m_ui->time_unit_combobox->setCurrentText(QString::fromStdString(m_preferences->getTimeUnitText()));
                connect(m_ui->time_unit_combobox, SIGNAL(currentIndexChanged(QString)), m_preferences.get(), SLOT(setTimeUnit(QString)));

                m_ui->angle_unit_combobox->addItems(QStringList() << "degree" << "radian");
                m_ui->angle_unit_combobox->setCurrentText(QString::fromStdString(m_preferences->getAngleUnitText()));
                connect(m_ui->angle_unit_combobox, SIGNAL(currentIndexChanged(QString)), m_preferences.get(), SLOT(setAngleUnit(QString)));
            }

            PreferencesWidget::~PreferencesWidget()
            {
                delete m_ui;
            }
        }
    }
}
