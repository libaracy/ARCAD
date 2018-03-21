#pragma once
#include "opencv2/core/core.hpp"
#include "display.h"

class ARWidget : public QWidget
{
    Q_OBJECT

public:
    ARWidget(QWidget* parent);
    ~ARWidget();

private slots:
    void receiveFrame(FramePtr frame);

private:
    void paintEvent(QPaintEvent *event) override;
    FramePtr m_frame = nullptr;

signals:
    void postFrame(FramePtr frame);
    void sizeChanged();
};


