#include "preferences_widget.hpp"
#include "ui_preferences_widget.h"

#include <QFileDialog>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            PreferencesWidget::PreferencesWidget(QWidget* parent)
                : QMainWindow(parent)
                , m_ui(new Ui::PreferencesWidget)
                , m_preferences_manager(PreferencesManager::getInstance())
            {
                m_ui->setupUi(this);

                m_ui->distance_units_combobox->addItems(QStringList() << "in" << "mm" << "cm");
                m_ui->distance_units_combobox->setCurrentText(m_preferences_manager->getDistanceUnitText());
                connect(m_ui->distance_units_combobox, SIGNAL(currentIndexChanged(QString)), m_preferences_manager.data()), SLOT(setDistanceUnit(QString));

                m_ui->time_units_combobox->addItems(QStringList() << "s" << "ms");
                m_ui->time_units_combobox->setCurrentText(m_preferences_manager->getTimeUnitText());
                connect(m_ui->time_units_combobox, SIGNAL(currentIndexChanged(QString)), m_preferences_manager.data()), SLOT(setTimeUnit(QString));
            }

            PreferencesWidget::~PreferencesWindow()
            {
                delete ui;
            }
        }
    }
}
