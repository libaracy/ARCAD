#pragma once
#include "DrawableData.h"


class Drawable : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QColor color READ getColor WRITE setColor)
    QColor getColor() { return QColor(modify().color[0], modify().color[1], modify().color[2]); }
    void setColor(QColor color) { modify().color = { color.red(), color.green(), color.blue() }; }

public:

    std::shared_ptr<const DrawableData> data()
    {
        modify();
        return m_data;
    }

protected:
    std::shared_ptr<DrawableData> m_data;

    template<typename T>
    T& _modify()
    {
        if (m_data == nullptr)
            m_data = std::make_shared<T>();
        return static_cast<T&>(*m_data);
    }

    virtual DrawableData& modify() = 0;
};


#define DEFINE_SIMPLE_PROPERTY(type, name, reference)\
Q_PROPERTY(type name READ get##name WRITE set##name)\
type get##name() { return modify().reference; }\
void set##name(type name) { modify().reference = name; }


class Point : public Drawable
{
    Q_OBJECT

    DEFINE_SIMPLE_PROPERTY(int, x, pos.x)
    DEFINE_SIMPLE_PROPERTY(int, y, pos.y)
    DEFINE_SIMPLE_PROPERTY(int, z, pos.z)
    DEFINE_SIMPLE_PROPERTY(int, thickness, thickness)

    PointData& modify()
    {
        return _modify<PointData>();
    }
};

class Cube : public Drawable
{
    Q_OBJECT

    DEFINE_SIMPLE_PROPERTY(int, x, pos.x)
    DEFINE_SIMPLE_PROPERTY(int, y, pos.y)
    DEFINE_SIMPLE_PROPERTY(int, z, pos.z)
    DEFINE_SIMPLE_PROPERTY(int, width, width)
    DEFINE_SIMPLE_PROPERTY(int, length, length)
    DEFINE_SIMPLE_PROPERTY(int, height, height)

    CubeData& modify()
    {
        return _modify<CubeData>();
    }
};


class LineCube : public Cube
{
    Q_OBJECT

    DEFINE_SIMPLE_PROPERTY(int, thickness, thickness)

    LineCubeData & modify()
    {
        return _modify<LineCubeData>();
    }
};


class PolyBase : public Drawable
{
    Q_OBJECT

    DEFINE_SIMPLE_PROPERTY(int, thickness, thickness)
    DEFINE_SIMPLE_PROPERTY(int, x1, vertexs[0][0])
    DEFINE_SIMPLE_PROPERTY(int, y1, vertexs[0][1])
    DEFINE_SIMPLE_PROPERTY(int, z1, vertexs[0][2])
    DEFINE_SIMPLE_PROPERTY(int, x2, vertexs[1][0])
    DEFINE_SIMPLE_PROPERTY(int, y2, vertexs[1][1])
    DEFINE_SIMPLE_PROPERTY(int, z2, vertexs[1][2])

    bool event(QEvent* e) override;

    PolyBaseData & modify() = 0;
};

class PolyLine : public PolyBase
{
    Q_OBJECT

    DEFINE_SIMPLE_PROPERTY(bool, closed, closed)

    PolyLineData & modify()
    {
        return _modify<PolyLineData>();
    }
};

class Plane : public PolyBase
{
    Q_OBJECT

    PlaneData & modify()
    {
        return _modify<PlaneData>();
    }
};
