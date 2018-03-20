#include "stdafx.h"
#include "MainWindow.h"
#include "ARWidget.h"

MainWindow::MainWindow()
{
    setFixedSize(1000, 1000);
    auto wnd = new ARWidget(this);
    wnd->setGeometry(geometry());
}
