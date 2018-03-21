// ARCADCore.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ARCore.h"
#define CORE_EXPORT
#include "display.h"
#include <mutex>

void demo(string videoname);
void testLocateZYJBoard();

void display()
{
    demo("calib2.mp4");
    //testLocateZYJBoard();
}

static std::function<void(FramePtr)> s_cb = nullptr;
static DisplayData s_data;
static std::mutex s_mutex;
static bool s_showFloor = false;
static bool s_showSpongeBob = false;

void setDisplayCallback(std::function<void(FramePtr)> cb)
{
    s_cb = cb;
}

void setDisplayData(DisplayData&& data)
{
    std::lock_guard<std::mutex> guard(s_mutex);
    s_data = data;
}

void toggleFloor()
{
    s_showFloor = !s_showFloor;
}

void toggleSpongeBob()
{
    s_showSpongeBob = !s_showSpongeBob;
}

void demo(string videoname) {
    cv::VideoCapture capture(1);
    ARCore arCore;
    arCore.init(116.0, false);
    arCore.addSpongeBob2VTK();

    //step1. calibration
    std::vector<cv::Mat> images4calib = {};
    arCore.readCalibrationResult();

    if (!capture.isOpened())
        return;

    CameraPose camPose = {};
    while (1) {
        auto pFrame = std::make_shared<cv::Mat>();
        auto& frame = *pFrame;

        capture >> frame;

        if (frame.empty())
        {
            printf("--(!) No captured frame -- Break!");
            return;
        }
        else {
            //step2. calculate frame
            if (arCore.calcCamPoseFromFrame(frame, camPose)) {
                //test spongebob
                arCore.setVTKWindowSize(frame);
                arCore.setVTKCamera(camPose);

                if (s_showSpongeBob)
                    arCore.combineVTK2Frame(frame, camPose);

                //step3. draw cube
                //arCore.drawZYJBoardCube(frame, 116, camPose);
                //arCore.drawPoint(frame, { 0, 0, -30 }, camPose, {255,255,0}, 3);
                //std::vector<cv::Vec3d> polyPoints = { {10,10,-10},  {21, 31, -10}, {42,31,-21}, {32, 43,-31}, {43, 55, -22}, 
                //{45,54, -34}, {65, 55, -44}, {75, 67, -44}, {100,150,-55}, {90, 90, -50},{70, 85, -45}, {65, 55, -30},
                //{54, 54, -44}, {44, 43, -33}, {33, 22, -32}, {21, 21, -15} };
                //arCore.drawPolyLine(frame, polyPoints, false, camPose, { 0, 255, 255 }, 1);
                //arCore.drawCurveLine(frame, polyPoints, false, camPose, { 255, 0, 255 }, 2);
                //test: draw any cube
                //arCore.drawCubeLine(frame, { 0.0, 116.0, -30.0 }, 50, 40, -30, camPose, {0,0,255}, 2);
                //arCore.drawCube(frame, { 116.0, 116.0, -40.0 }, 50, 50, -30, camPose, { 0,0,255 });

                if (s_showFloor)
                {
                    const cv::Vec3d origin(-20, 171, 0);
                    constexpr int size = 3;
                    constexpr int step = 50;

                    for (int i = 0; i <= size; i++)
                    {
                        cv::Vec3d startp = { origin[0], origin[1], origin[2] + step * i };
                        cv::Vec3d endp = { origin[0] + step * size, origin[1], origin[2] + step * i };
                        arCore.drawPolyLine(frame, { startp, endp }, false, camPose);
                    }
                    for (int i = 0; i <= size; i++)
                    {
                        cv::Vec3d startp = { origin[0] + step * i, origin[1], origin[2] };
                        cv::Vec3d endp = { origin[0] + step * i, origin[1], origin[2] + step * size };
                        arCore.drawPolyLine(frame, { startp, endp }, false, camPose);
                    }
                }

                std::lock_guard<std::mutex> guard(s_mutex);
                for (auto& item : s_data)
                {
                    cv::Scalar color = { (double)item->color[2], (double)item->color[1], (double)item->color[0] };

                    switch (item->type)
                    {
                    case DrawableType::ePoint:
                    {
                        auto pData = static_cast<const PointData*>(item.get());
                        arCore.drawPoint(frame, pData->pos, camPose, color, pData->thickness);
                    }
                    case DrawableType::eCube:
                    {
                        auto pData = static_cast<const CubeData*>(item.get());
                        arCore.drawCube(frame, pData->pos, pData->width, pData->length, pData->height, camPose, color);
                        break;
                    }
                    case DrawableType::eLineCube:
                    {
                        auto pData = static_cast<const LineCubeData*>(item.get());
                        arCore.drawCubeLine(frame, pData->pos, pData->width, pData->length, pData->height, camPose, color, pData->thickness);
                        break;
                    }
                    case DrawableType::ePolyLine:
                    {
                        auto pData = static_cast<const PolyLineData*>(item.get());
                        arCore.drawPolyLine(frame, pData->vertexs, pData->closed, camPose, color, pData->thickness);
                        break;
                    }
                    default:
                        throw exception("impossible");
                    }
                }
            }
            if (s_cb) {
                arCore.undistort(frame);
                cv::cvtColor(frame, frame, CV_BGR2RGB);
                s_cb(pFrame);
            }
        }
        cv::waitKey(10);
    }
}

void testLocateZYJBoard() {
    cv::Mat src = cv::imread("notRecog.png", CV_LOAD_IMAGE_UNCHANGED);
    ARCore arCore;
    std::vector<cv::Point2d> sortedCorners = {};
    arCore.locateZYJBoard(src, sortedCorners, true);
}