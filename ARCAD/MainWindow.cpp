#include "stdafx.h"
#include "MainWindow.h"
#include "ARWidget.h"

MainWindow::MainWindow()
{
    setFixedSize(800, 600);

    auto wnd = new ARWidget(this);
    wnd->setGeometry(geometry());
}
