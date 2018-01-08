#include "course_settings_widget.hpp"
#include "ui_course_settings_widget.h"

#include <QInputDialog>

#include "point_list_widget_item.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            CourseSettingsWidget::CourseSettingsWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::CourseSettingsWidget)
                , m_settings(Settings::getInstance())
                , m_preferences(Preferences::getInstance())
                , m_compute_thread(ComputeThread::getInstance())
            {
                m_ui->setupUi(this);
                connect(m_ui->options, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateCourse(QString)));

                /*
                m_ui->point_widget->setSelectionMode(QAbstractItemView::SingleSelection);
                m_ui->point_widget->setDragEnabled(true);
                m_ui->point_widget->setAcceptDrops(true);
                m_ui->point_widget->setDropIndicatorShown(true);
                */

                connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(addCourse()));
                connect(m_ui->remove, SIGNAL(clicked(bool)), this, SLOT(removeCourse()));
                connect(m_ui->num_points, SIGNAL(textChanged(QString)), this, SLOT(updateNumPoints()));

                m_ui->num_points->setEnabled(false);
            }

            void CourseSettingsWidget::setCourseOptions(const std::vector<std::string>& options)
            {
                if(options.size())
                {
                    QStringList list;
                    for (const std::string& item : options)
                    {
                        list << QString::fromStdString(item);
                    }
                    m_ui->options->addItems(list);
                    updateCourse(QString::fromStdString(options.front()));
                }
            }

            std::string CourseSettingsWidget::course() const
            {
                if(m_current)
                {
                    return m_current->name();
                }
                return "";
            }

            geometry::Polygon CourseSettingsWidget::polygon()
            {
                return m_polygon;
            }

            void CourseSettingsWidget::updateCourse(QString name)
            {
                m_current = m_settings->course(name.toStdString());
                if(m_current)
                {
                    units::Distance d_unit = m_preferences->getDistanceUnit();
                    int np = 0;
                    m_ui->point_widget->clear();
                    for(const geometry::Point& p: m_current->get<geometry::Polygon>("points"))
                    {
                        PointListWidgetItem* plwi = new PointListWidgetItem(m_ui->point_widget);
                        plwi->show();
                        QListWidgetItem* lwi = new QListWidgetItem(m_ui->point_widget);
                        lwi->setSizeHint(plwi->sizeHint());
                        plwi->setX(p.x().to(d_unit));
                        plwi->setY(p.y().to(d_unit));
                        m_ui->point_widget->addItem(lwi);
                        m_ui->point_widget->setItemWidget(lwi, plwi);
                        connect(plwi, SIGNAL(modified()), this, SLOT(updatePolygon()));
                        np++;
                    }
                    updatePolygon();
                    m_ui->num_points->setEnabled(true);
                    m_ui->num_points->setText(QString::number(np));
                }
            }

            void CourseSettingsWidget::addCourse()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Course"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if(ok && !name.isEmpty())
                {
                    m_settings->addCourse(name.toStdString());
                    m_ui->options->addItem(name);
                    updateCourse(name);
                }
            }

            void CourseSettingsWidget::removeCourse()
            {
                int index = m_ui->options->currentIndex();
                if(index >= 0)
                {
                    m_settings->removeCourse(m_current->name());
                    m_ui->options->removeItem(index);
                }
            }

            void CourseSettingsWidget::updateNumPoints()
            {
                bool ok = false;
                int num_points = m_ui->num_points->text().toInt(&ok);
                if(ok)
                {
                    int count = m_ui->point_widget->count();
                    if(num_points > count)
                    {
                        // Increase the number of rows
                        for(int i = count; i < num_points; i++)
                        {
                            PointListWidgetItem* plwi = new PointListWidgetItem(m_ui->point_widget);
                            plwi->show();
                            QListWidgetItem* lwi = new QListWidgetItem(m_ui->point_widget);
                            lwi->setSizeHint(plwi->sizeHint());
                            m_ui->point_widget->addItem(lwi);
                            m_ui->point_widget->setItemWidget(lwi, plwi);
                            connect(plwi, SIGNAL(modified()), this, SLOT(updatePolygon()));
                        }
                    }
                    else if(num_points < count)
                    {
                        // Decrease number of rows
                        for(int i = count - 1; i >= num_points; i--)
                        {
                            QListWidgetItem* lwi = m_ui->point_widget->item(i);
                            delete m_ui->point_widget->itemWidget(lwi);
                            delete m_ui->point_widget->takeItem(i);
                        }
                        updatePolygon();
                    }
                }
            }

            void CourseSettingsWidget::updatePolygon()
            {
                geometry::Polygon polygon;
                units::Distance d_unit = m_preferences->getDistanceUnit();
                for(int i = 0, end = m_ui->point_widget->count(); i < end; i++)
                {
                    QListWidgetItem* lwi = m_ui->point_widget->item(i);
                    PointListWidgetItem* plwi = static_cast<PointListWidgetItem*>(m_ui->point_widget->itemWidget(lwi));

                    geometry::Point p(plwi->x() * d_unit, plwi->y() * d_unit);
                    polygon += p;
                }
                m_current->set<geometry::Polygon>("points", polygon);
                m_ui->polygon_widget->updatePolygon(polygon);
                m_polygon = polygon;
                emit polygonUpdated();
            }
        }
    }
}
