// TRACE — operator console entry point.
#include <QApplication>

#include "console_window.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setApplicationName("TRACE");
    app.setOrganizationName("TRACE");

    ConsoleWindow window;
    window.show();
    return app.exec();
}
