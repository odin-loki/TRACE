// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

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
