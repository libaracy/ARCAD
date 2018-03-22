#pragma once
#include "opencv2/core/core.hpp"
#include <array>


enum class DrawableType
{
    eInvalid,
    ePoint,
    ePolyLine,
    ePlane,
    eCube,
    eLineCube
};


struct DrawableData
{
    std::array<int, 3> color = { 0, 255, 0 };
    DrawableType type = DrawableType::eInvalid;
};


struct PointData : DrawableData
{
    cv::Point3d pos;
    int thickness = 5;
    PointData()
    {
        type = DrawableType::ePoint;
    }
};


struct CubeData : public DrawableData
{
    cv::Point3d pos;
    double width = 50;
    double length = 50;
    double height = -50;
    CubeData()
    {
        type = DrawableType::eCube;
    }
};


struct LineCubeData : public CubeData
{
    int thickness = 2;
    LineCubeData()
    {
        type = DrawableType::eLineCube;
    }
};


struct PolyBaseData : public DrawableData
{
    std::vector<cv::Vec3d> vertexs = { {0, 0, 0}, {20, 20, -20} };
    int thickness = 2;
};

struct PolyLineData : public PolyBaseData
{
    bool closed = false;
    PolyLineData()
    {
        type = DrawableType::ePolyLine;
    }
};

struct PlaneData : public PolyBaseData
{
    PlaneData()
    {
        type = DrawableType::ePlane;
    }
};