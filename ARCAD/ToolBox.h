#pragma once
#include "Drawable.h"

class ToolBox : public QWidget
{
    Q_OBJECT

public:
    ToolBox(QWidget* parent);
    ~ToolBox();

signals:
    void requestAddDrawable(Drawable* obj);
};

