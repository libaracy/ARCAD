#include "stdafx.h"
#include "ObjectCollection.h"
#include "display.h"


ObjectCollection::ObjectCollection(QWidget* parent)
    : QListWidget(parent)
{
    connect(this, &ObjectCollection::currentItemChanged,
        [this](QListWidgetItem* current, QListWidgetItem* previous)
    {
        currentObjectChanged(current == nullptr ? nullptr : findChild<Drawable*>(current->text(), Qt::FindDirectChildrenOnly));
    });

    setSelectionMode(QAbstractItemView::ExtendedSelection);
}

void ObjectCollection::contextMenuEvent(QContextMenuEvent* e)
{
    auto items = selectedItems();
    if (items.isEmpty())
        return;

    QMenu menu(this);
    auto del = menu.addAction("Delete");
    if (del == menu.exec(mapToGlobal(e->pos())))
    {
        for (auto item : items)
        {
            if (auto obj = findChild<Drawable*>(item->text(), Qt::FindDirectChildrenOnly))
            {
                delete item;
                delete obj;
            }
        }
        updateDisplayData();
    }
}

ObjectCollection::~ObjectCollection()
{
}

void ObjectCollection::addDrawable(Drawable* obj)
{
    assert(obj->objectName().isEmpty());

    obj->setParent(this);

    QString prefix = obj->metaObject()->className();
    prefix += "_";

    for (int i = 0; ; i++)
    {
        QString name = prefix + QString::number(i);

        if (nullptr == findChild<Drawable*>(name, Qt::FindDirectChildrenOnly))
        {
            obj->setObjectName(name);

            auto item = new QListWidgetItem(name);
            addItem(item);
            clearSelection();
            setCurrentItem(item);
            updateDisplayData();
            break;
        }
    }
}

void ObjectCollection::updateDisplayData()
{
    DisplayData data;

    for (auto pObj : findChildren<Drawable*>(QString(), Qt::FindDirectChildrenOnly))
    {
        data.push_back(pObj->data());
    }

    setDisplayData(std::move(data));
}