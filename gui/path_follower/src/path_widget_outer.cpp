#include "path_follower_gui/path_widget_outer.hpp"
#include "ui_path_widget_outer.h"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {

                PathWidgetOuter::PathWidgetOuter(QWidget* parent)
                    : QWidget(parent)
                    , m_ui(new Ui::PathWidgetOuter)
                {
                    m_ui->setupUi(this);
                }

                PathWidgetOuter::~PathWidgetOuter()
                {
                    delete m_ui;
                }

                void PathWidgetOuter::setAnimate()
                {

                }

            }
        }
    }
}
