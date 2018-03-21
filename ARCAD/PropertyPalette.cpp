#include "stdafx.h"
#include "PropertyPalette.h"
#include "Drawable.h"


PropertyPalette::PropertyPalette(QWidget* parent)
    : QTableWidget(parent)
{
    setColumnCount(2);
    setHorizontalHeaderLabels({ "Property", "Value" });
    verticalHeader()->hide();
    horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    connect(this, &QTableWidget::cellChanged, [this](int row, int column)
    {
        if (column != 1)
            return;
        QString name = item(row, 0)->text();
        QVariant value = item(row, 1)->data(Qt::DisplayRole);
        m_obj->setProperty(name.toStdString().c_str(), value);
    });
}


PropertyPalette::~PropertyPalette()
{
}

void PropertyPalette::setObject(QObject* obj)
{
    m_obj = obj;

    if (m_obj == nullptr)
    {
        setRowCount(0);
        return;
    }

    auto mo = m_obj->metaObject();
    setRowCount(mo->propertyCount());

    auto setRowContent = [this](int row, const QString& key, const QVariant& value)
    {
        auto item = new QTableWidgetItem(key);
        setItem(row, 0, item);
        item->setFlags(Qt::ItemIsEnabled);

        item = new QTableWidgetItem();
        item->setData(Qt::DisplayRole, value);
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable);
        setItem(row, 1, item);
    };

    for (int i = 0; i < mo->propertyCount(); i++)
    {
        QString name = mo->property(i).name();
        if (name == "objectName") name = "name";

        QVariant value = mo->property(i).read(m_obj);

        setRowContent(i, name, value);
    }

    if (typeid(*m_obj) == typeid(PolyLine))
    {
        int j = 3;
        for (; ; j++)
        {
            QString x = "x" + QString::number(j);
            QString y = "y" + QString::number(j);
            QString z = "z" + QString::number(j);

            QVariant vx = m_obj->property(x.toStdString().c_str());
            QVariant vy = m_obj->property(y.toStdString().c_str());
            QVariant vz = m_obj->property(z.toStdString().c_str());

            if (vx.isValid() && vy.isValid() && vz.isValid())
            {
                setRowCount(rowCount() + 3);

                setRowContent(rowCount() - 3, x, vx);
                setRowContent(rowCount() - 2, y, vy);
                setRowContent(rowCount() - 1, z, vz);
            }
            else
            {
                break;
            }
        }

        setRowCount(rowCount() + 2);

        setSpan(rowCount() - 2, 0, 1, 2);
        auto add = new QPushButton("Add Vertex", this);
        setCellWidget(rowCount() - 2, 0, add);

        setSpan(rowCount() - 1, 0, 1, 2);
        auto del = new QPushButton("Del Vertex", this);
        setCellWidget(rowCount() - 1, 0, del);

        del->setEnabled(j > 3);

        connect(add, &QPushButton::clicked, [=]() {
            m_obj->setProperty(("x" + std::to_string(j)).c_str(), 0);
            m_obj->setProperty(("y" + std::to_string(j)).c_str(), 0);
            m_obj->setProperty(("z" + std::to_string(j)).c_str(), 0);
            setObject(m_obj);
        });

        connect(del, &QPushButton::clicked, [=]() {
            m_obj->setProperty(("x" + std::to_string(j - 1)).c_str(), QVariant());
            m_obj->setProperty(("y" + std::to_string(j - 1)).c_str(), QVariant());
            m_obj->setProperty(("z" + std::to_string(j - 1)).c_str(), QVariant());
            setObject(m_obj);
        });
    }
}