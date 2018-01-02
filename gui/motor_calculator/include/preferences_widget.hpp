#ifndef PREFERENCES_WIDGET_HPP
#define PREFERENCES_WIDGET_HPP

#include <memory>

#include <QMainWindow>
#include <QWidget>
#include <QCloseEvent>

#include "preferences.hpp"

namespace Ui
{
    class PreferencesWidget;
}

namespace rip
{
    namespace gui
    {
        namespace motorcalc
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

                std::shared_ptr<Preferences> m_preferences;
            };
        }
    }
}

#endif // PREFERENCES_WIDGET_HPP
