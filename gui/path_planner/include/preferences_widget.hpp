#ifndef PREFERENCES_WINDOW_HPP
#define PREFERENCES_WINDOW_HPP

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
                Ui::PreferencesWidget *ui;

                std::shared_ptr<PreferencesManager> m_preferences_manager;
            };
        }
    }
}

#endif // PREFERENCES_WINDOW_HPP
