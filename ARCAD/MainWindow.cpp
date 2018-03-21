#include "stdafx.h"
#include "MainWindow.h"
#include "ARWidget.h"
#include "PropertyPalette.h"
#include "ObjectCollection.h"
#include "ToolBox.h"

MainWindow::MainWindow()
{
    auto wnd = new ARWidget(this);
    setCentralWidget(wnd);
    connect(wnd, &ARWidget::sizeChanged, this, &MainWindow::adjustSize);

    auto dock = new QDockWidget("Property Palette", this);
    auto pp = new PropertyPalette(this);
    dock->setWidget(pp);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    dock = new QDockWidget("Tool Box", this);
    auto tb = new ToolBox(this);
    dock->setWidget(tb);
    addDockWidget(Qt::LeftDockWidgetArea, dock);

    dock = new QDockWidget("Object Collection", this);
    auto oc = new ObjectCollection(this);
    dock->setWidget(oc);
    addDockWidget(Qt::LeftDockWidgetArea, dock);

    connect(oc, &ObjectCollection::currentObjectChanged, pp, &PropertyPalette::setObject);
    connect(tb, &ToolBox::requestAddDrawable, oc, &ObjectCollection::addDrawable);
}
