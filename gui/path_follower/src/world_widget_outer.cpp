#include "path_follower_gui/world_widget_outer.hpp"
#include "ui_world_widget_outer.h"
#include "path_follower_gui/storage.hpp"

#include <QInputDialog>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                WorldWidgetOuter::WorldWidgetOuter(QWidget* parent)
                    : QWidget(parent)
                    , m_ui(new Ui::WorldWidgetOuter)
                {
                    m_ui->setupUi(this);

                    QStringList names;
                    for (const std::string& name : Storage::getInstance()->worldNames())
                    {
                        names << QString::fromStdString(name);
                    }
                    m_ui->options->addItems(names);
                    if(m_ui->options->currentIndex() > -1)
                    {
                        Storage::getInstance()->selectWorld(m_ui->options->currentText());
                    }

                    connect(m_ui->options, SIGNAL(currentTextChanged(QString)), Storage::getInstance().get(), SLOT(selectWorld(QString)));
                    connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(add()));
                    connect(m_ui->remove, SIGNAL(clicked(bool)), this, SLOT(remove()));
                    connect(Storage::getInstance().get(), SIGNAL(worldOptionsChanged()), this, SLOT(worldOptionsChanged()));
                }

                WorldWidgetOuter::~WorldWidgetOuter()
                {
                    delete m_ui;
                }

                void WorldWidgetOuter::worldOptionsChanged()
                {
                    disconnect(m_ui->options, SIGNAL(currentTextChanged(QString)), this, SLOT(worldChanged(QString)));

                    QStringList names;
                    for (const std::string& name : Storage::getInstance()->worldNames())
                    {
                        names << QString::fromStdString(name);
                    }

                    m_ui->options->clear();
                    m_ui->options->addItems(names);
                    m_ui->options->setCurrentText(QString::fromStdString(Storage::getInstance()->selectedWorldName()));

                    connect(m_ui->options, SIGNAL(currentTextChanged(QString)), this, SLOT(worldChanged(QString)));
                }

                void WorldWidgetOuter::add()
                {
                    bool ok;
                    QString name = QInputDialog::getText(this, tr("Add World"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                    if (ok && !name.isEmpty())
                    {
                        Storage::getInstance()->addWorld(name.toStdString());
                    }
                }

                void WorldWidgetOuter::remove()
                {
                    Storage::getInstance()->removeWorld(m_ui->options->currentText().toStdString());
                }
            }
        }
    }
}
