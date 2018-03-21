#include "stdafx.h"
#include "PropertyPalette.h"

//class PropertyPaletteModel : public QAbstractTableModel
//{
//    QObject* m_obj;
//
//public:
//    PropertyPaletteModel(QObject* parent = nullptr)
//        : QAbstractTableModel(parent)
//    {
//    }
//    void setDisplayObject(QObject* obj)
//    {
//        m_obj = obj;
//        layoutChanged();
//    }
//    int rowCount(const QModelIndex& parent = QModelIndex()) const override
//    {
//        return 2;
//    }
//    int columnCount(const QModelIndex& parent = QModelIndex()) const override
//    {
//        return 2;
//    }
//};

PropertyPalette::PropertyPalette(QWidget* parent)
    : QTableWidget(parent)
{
    setColumnCount(2);
    setHorizontalHeaderLabels({ "Property", "Value" });
    verticalHeader()->hide();
    horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    //setSelectionMode(QAbstractItemView::NoSelection);

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

    for (int i = 0; i < mo->propertyCount(); i++)
    {
        QString name = mo->property(i).name();
        if (name == "objectName") name = "name";

        QVariant value = mo->property(i).read(m_obj);

        setItem(i, 0, new QTableWidgetItem(name));
        auto item = new QTableWidgetItem();
        item->setData(Qt::DisplayRole, value);
        setItem(i, 1, item);
    }
}