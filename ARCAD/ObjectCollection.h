#pragma once
#include "Drawable.h"

class ObjectCollection : public QListWidget
{
    Q_OBJECT

public:
    ObjectCollection(QWidget* parent = nullptr);
    ~ObjectCollection();

    void addDrawable(Drawable*);

signals:
    void currentObjectChanged(QObject*);

private:
    void contextMenuEvent(QContextMenuEvent* e) override;

    void updateDisplayData();
};

