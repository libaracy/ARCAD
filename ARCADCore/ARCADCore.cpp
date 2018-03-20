// ARCADCore.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ARCore.h"
#define CORE_EXPORT
#include "display.h"

void demo(string videoname);
void testLocateZYJBoard();

void display()
{
    demo("calib2.mp4");
    //testLocateZYJBoard();
}

static std::function<void(FramePtr)> g_cb = nullptr;

void setDisplayCallback(std::function<void(FramePtr)> cb)
{
    g_cb = cb;
}

void demo(string videoname) {
    cv::VideoCapture capture(videoname);
    ARCore arCore;
    arCore.init(116.0);

    //step1. calibration
    std::vector<cv::Mat> images4calib = {};
    //arCore.grabCalibrationFrames(videoname, images4calib);
    /*if (!arCore.calibrationNow(images4calib)) {
        std::cout << "calibration failed!" << std::endl;
        return;
    }*/
    //arCore.saveCalibrationResult();
    arCore.readCalibrationResult();

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
            CameraPose camPose = {};
            if (arCore.calcCamPoseFromFrame(frame, camPose)) {
                //step3. draw cube
                //arCore.drawZYJBoardCube(frame, 116, camPose);
                arCore.drawPoint(frame, { 0, 0, -30 }, camPose, {255,255,0}, 3);
                std::vector<cv::Vec3d> polyPoints = { {10,10,-10},  {21, 31, -10}, {42,31,-21}, {32, 43,-31}, {43, 55, -22}, 
                {45,54, -34}, {65, 55, -44}, {75, 67, -44}, {100,150,-55}, {90, 90, -50},{70, 85, -45}, {65, 55, -30},
                {54, 54, -44}, {44, 43, -33}, {33, 22, -32}, {21, 21, -15} };
                arCore.drawPolyLine(frame, polyPoints, false, camPose, { 0, 255, 255 }, 1);
                arCore.drawCurveLine(frame, polyPoints, false, camPose, { 255, 0, 255 }, 2);
                //test: draw any cube
                arCore.drawCubeLine(frame, { 0.0, 116.0, -30.0 }, 50, 40, -30, camPose, {0,0,255}, 2);
                arCore.drawCube(frame, { 116.0, 116.0, -40.0 }, 50, 50, -30, camPose, { 0,0,255 });
            }
            if (g_cb) {
                cv::cvtColor(frame, frame, CV_BGR2RGB);
                g_cb(pFrame);
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