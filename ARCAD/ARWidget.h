#pragma once

class ARWidget : public QWidget
{
    Q_OBJECT

public:
    ARWidget(QWidget* parent);
    ~ARWidget();

private:
    void paintEvent(QPaintEvent *event) override;
};

