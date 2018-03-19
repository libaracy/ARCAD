#include "stdafx.h"
#include "ARWidget.h"


ARWidget::ARWidget(QWidget* parent)
    :QWidget(parent)
{
}


ARWidget::~ARWidget()
{
}


void ARWidget::paintEvent(QPaintEvent * event)
{
    QPainter painter(this);

    painter.drawRect(100, 100, 100, 100);
}