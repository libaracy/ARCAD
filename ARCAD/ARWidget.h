#pragma once
#include "opencv2/core/core.hpp"
#include "display.h"

class ARWidget : public QWidget
{
    Q_OBJECT

public:
    ARWidget(QWidget* parent);
    ~ARWidget();

private:
    void paintEvent(QPaintEvent *event) override;
    FramePtr m_frame = nullptr;
};

