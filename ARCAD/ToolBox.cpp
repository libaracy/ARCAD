#include "stdafx.h"
#include "ToolBox.h"
#include "display.h"

template <typename T>
static QPushButton* createButton(ToolBox* toolbox)
{
    QString text = T::staticMetaObject.className();
    text = "Add " + text;
    QPushButton* btn = new QPushButton(text, toolbox);
    QObject::connect(btn, &QPushButton::clicked, [=]() {
        toolbox->requestAddDrawable(new T);
    });
    return btn;
}

ToolBox::ToolBox(QWidget* parent)
    : QWidget(parent)
{
    auto layout = new QVBoxLayout(this);
    setLayout(layout);

    layout->addWidget(createButton<Point>(this));
    layout->addWidget(createButton<Cube>(this));
    layout->addWidget(createButton<LineCube>(this));
    layout->addWidget(createButton<PolyLine>(this));

    QPushButton* btn = new QPushButton("Toggle Floor", this);
    connect(btn, &QPushButton::clicked, toggleFloor);
    layout->addWidget(btn);

    btn = new QPushButton("Toggle Coordinate", this);
    connect(btn, &QPushButton::clicked, toggleCoordinate);
    layout->addWidget(btn);

    btn = new QPushButton("Toggle Animation", this);
    connect(btn, &QPushButton::clicked, toggleAnimate);
    layout->addWidget(btn);

    btn = new QPushButton("Toggle AutoCAD", this);
    connect(btn, &QPushButton::clicked, toggleAutoCAD);
    layout->addWidget(btn);

    btn = new QPushButton("Toggle SpongeBob", this);
    connect(btn, &QPushButton::clicked, toggleSpongeBob);
    layout->addWidget(btn);
}


ToolBox::~ToolBox()
{
}
