#ifndef PREFERENCES_WIDGET_HPP
#define PREFERENCES_WIDGET_HPP

#include <memory>

#include <QMainWindow>
#include <QWidget>
#include <QCloseEvent>

#include "preferences_manager.hpp"

namespace Ui
{
    class PreferencesWidget;
}

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            /**
             * Window that allows the user to change his/her preferences
             */
            class PreferencesWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit PreferencesWidget(QWidget* parent = nullptr);

                ~PreferencesWidget();
            private:
                Ui::PreferencesWidget* m_ui;

                std::shared_ptr<Preferences> m_preferences_manager;
            };
        }
    }
}

#endif // PREFERENCES_WIDGET_HPP
