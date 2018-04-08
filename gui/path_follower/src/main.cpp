#include <QApplication>
#include <QFile>
#include <QTextStream>
#include "path_follower_gui/main_window.hpp"
#include "path_follower_gui/storage.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    Q_INIT_RESOURCE(icons);
    Q_INIT_RESOURCE(style);

    QFile f(":qdarkstyle/style.qss");
    if (!f.exists())
    {
        printf("Unable to set stylesheet, file not found\n");
    }
    else
    {
        f.open(QFile::ReadOnly | QFile::Text);
        QTextStream ts(&f);
        qApp->setStyleSheet(ts.readAll());
    }

    rip::navigation::pathfollower::gui::Storage::getInstance()->load();

    rip::navigation::pathfollower::gui::MainWindow window;
    window.showMaximized();
    int value = app.exec();
    rip::navigation::pathfollower::gui::Storage::getInstance()->save();
    return value;
}
