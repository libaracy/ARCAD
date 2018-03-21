#pragma once

class PropertyPalette : public QTableWidget
{
public:
    PropertyPalette(QWidget* parent = nullptr);
    ~PropertyPalette();

    void setObject(QObject* obj);

private:
    QObject* m_obj = nullptr;
};

