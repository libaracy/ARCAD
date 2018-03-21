#include "stdafx.h"
#include "ToolBox.h"

template <typename T>
static QPushButton* createButton(ToolBox* toolbox)
{
    QString text = T::staticMetaObject.className();
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
}


ToolBox::~ToolBox()
{
}
