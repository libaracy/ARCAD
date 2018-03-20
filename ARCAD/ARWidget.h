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
    void receiveFrame(FramePtr frame);

    FramePtr m_frame;

signals:
    void postFrame(FramePtr frame);
};

