#include "stdafx.h"
#include "ARWidget.h"

Q_DECLARE_METATYPE(FramePtr)

ARWidget::ARWidget(QWidget* parent)
    :QWidget(parent)
{
    qRegisterMetaType<FramePtr>();

    connect(this, &ARWidget::postFrame, this, &ARWidget::receiveFrame);

    setDisplayCallback([this](FramePtr frame) {
        postFrame(frame);
    });

    std::thread(display).detach();
}

void ARWidget::receiveFrame(FramePtr frame)
{
    m_frame = frame;

    QSize frameSize(m_frame->cols, m_frame->rows);
    if (frameSize != size())
    {
        setFixedSize(frameSize);
        sizeChanged();
    }

    update();
}

ARWidget::~ARWidget()
{
}


void ARWidget::paintEvent(QPaintEvent * event)
{
    if (m_frame)
    {
        QPainter painter(this);
        QImage img(m_frame->data, m_frame->cols, m_frame->rows, QImage::Format_RGB888);
        painter.drawImage(0, 0, img);
    }
}