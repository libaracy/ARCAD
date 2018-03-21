#include "stdafx.h"
#include "ARCore.h"

/////////////////////////////////////////
// description: constructor
/////////////////////////////////////////
ARCore::ARCore() {}

/////////////////////////////////////////
// description: init
/////////////////////////////////////////
void ARCore::init(double zyjBoardLength, bool showVTK) {
    setZYJBoard(zyjBoardLength);
    initVTK({ 1280,720 }, showVTK);
}

/////////////////////////////////////////
// description: calibrate the camera
// params - frames: calibration plate photos
/////////////////////////////////////////
bool ARCore::calibrationNow(const vector<cv::Mat> & frames) {
    if (frames.size() < 10)
        return false;

    std::cout << "----------------------start calibration-----------------" << std::endl;
    std::cout << std::endl;
    //1. get the basic information
    cv::Size image_size;                                        /* the size of image */
    cv::Size board_size = cv::Size(6, 8);                       /* the corner number on very row or column of calibration plate */
    std::vector<cv::Point2f> image_points_buf;                  /* the cache for corner point */
    std::vector<std::vector<cv::Point2f>> image_points_seq;     /* save every detected corner point */

    image_size.width = frames[0].cols;
    image_size.height = frames[0].rows;
    std::cout << "image_size.width = " << image_size.width << std::endl;
    std::cout << "image_size.height = " << image_size.height << std::endl;
    for (const Mat & tmp : frames) {
        if (!findChessboardCorners(tmp, board_size, image_points_buf))
        {
            std::cout << "can not find chessboard corners!\n";
            return false;
        }
        else {
            cv::Mat view_gray;
            cvtColor(tmp, view_gray, CV_RGB2GRAY);
            /* sub-pixel accurancy */
            find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5));
            image_points_seq.push_back(image_points_buf);
            cv::drawChessboardCorners(view_gray, board_size, image_points_buf, false);
            cv::imshow("Camera Calibration", view_gray);
            cv::waitKey(200);
        }
    }
    cv::destroyWindow("Camera Calibration");
    //2. calibration
    cv::Size square_size = cv::Size(28, 28);  /* the hand measured scale of grid in the plate */
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<int> point_counts;
    std::vector<cv::Mat> tvecsMat;  /* the rotate vector of every image */
    std::vector<cv::Mat> rvecsMat; /* the move vector of every image */
    int i, j, t;
    for (t = 0; t<frames.size(); t++)
    {
        std::vector<cv::Point3f> tempPointSet;
        for (i = 0; i<board_size.height; i++)
        {
            for (j = 0; j<board_size.width; j++)
            {
                cv::Point3f realPoint;
                /* assume plate is on the z=0 plane */
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }
    /* initilize the number of the corner points, assume every image could see the complete calibration plate */
    for (i = 0; i<frames.size(); i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    cv::calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    double total_err = 0.0; /* the total mean error of whole images */
    double err = 0.0; /* the mean error of single image */
    std::vector<cv::Point2f> image_points2; /* save again */
    std::cout << "\t the calibration error of every image:\n";
    for (i = 0; i<frames.size(); i++)
    {
        std::vector<cv::Point3f> tempPointSet = object_points[i];
        projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
        std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
        cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
        cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
        for (int j = 0; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
        total_err += err /= point_counts[i];
        std::cout << "the " << i + 1 << " image's mean error: " << err << " pixels" << std::endl;
    }
    std::cout << "total mean error: " << total_err / frames.size() << " pixels" << std::endl;
    std::cout << std::endl;
    std::cout << "----------------------calibration complete-----------------" << std::endl;

    return true;
}

/////////////////////////////////////////
// description: extract frames for calibration from video
// params - images4calib: calibration plate photos
/////////////////////////////////////////
bool ARCore::grabCalibrationFrames(string videoname, std::vector<cv::Mat> & images4calib) {
    cv::VideoCapture capture(videoname);
    int frameNum = 0;
    while (1) {
        cv::Mat frame;
        capture >> frame;
        if (frame.empty())
        {
            std::cout << "--(!) No captured frame -- Break!" << std::endl;
            return false;
        }
        else {
            if (frameNum <= 100) {
                if (frameNum == 10 || frameNum == 20 || frameNum == 30 || frameNum == 40 || frameNum == 50 ||
                    frameNum == 60 || frameNum == 70 || frameNum == 80 || frameNum == 90 || frameNum == 100) {
                    images4calib.push_back(frame);
                }
                frameNum += 1;
            }
            else {
                break;
            }
        }
    }
    if (frameNum != 101)
        return false;
    return true;
}

/////////////////////////////////////////
// description: calculate the camera pose in world coordinate
// params - frame: the frame which contains the zyjBoard
//          camPose: the camera pose
/////////////////////////////////////////
bool ARCore::calcCamPoseFromFrame(const cv::Mat & frame, CameraPose & camPose) {
    cv::Mat src = frame.clone();

    //1. locate the zyj board
    std::vector<cv::Point2d> sortedCorners = {};
    if (!locateZYJBoard(frame, sortedCorners, false))
        return false;

    //2. calculate the position and pose of the camera
    if (zyjBoardCorner3dCoords.size() != 4)
        return false;
    CameraPose tmpPose;
    if (solvePnP(zyjBoardCorner3dCoords, sortedCorners, cameraMatrix, distCoeffs, tmpPose.rvec, tmpPose.tvec))//get pose info
    {
        //std::cout << "sortedCorners:  " << sortedCorners << std::endl;
        camPose.setPose(tmpPose.rvec, tmpPose.tvec);
    }
    else {
        return false;
    }

    return true;
}

/////////////////////////////////////////
// description: get the zyjboard's corners pixel position in frame
// params - frame: the frame which contains the zyjBoard
//          sortedCorners: the corners' pose (rectangle, triangle, none, circle)
/////////////////////////////////////////
bool ARCore::locateZYJBoard(const cv::Mat & frame, cv::vector<cv::Point2d> & sortedCorners, bool progress) {
    cv::Mat src = frame.clone();

    //1. find the zyjboard
    cv::Mat src_gray1, src_gray2;
    cvtColor(src, src_gray1, CV_BGR2GRAY);
    threshold(src_gray1, src_gray1, 55, 255, THRESH_BINARY);
    /*Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    erode(src_gray1, src_gray1, element);
    dilate(src_gray1, src_gray1, element);*/
    if (progress) {
        cv::imshow("gray1", src_gray1);
        cv::waitKey(0);
    }
    bitwise_not(src_gray1, src_gray2);
    if (progress) {
        cv::imshow("gray2", src_gray2);
        cv::waitKey(0);
    }
    cv::vector<cv::vector<cv::Point>> contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::vector<cv::Mat> contoursMat(6000);
    cv::Mat hierarchyMat;
    cv::vector<cv::Point2d> MarkerContour = {};
    cv::vector<int> innerIndexRecord = {};
    findContours(src_gray2, contoursMat, hierarchyMat, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    contoursMat2Vec(contoursMat, contours, hierarchyMat, hierarchy);
    cv::vector<cv::Point> approxCurve = {};
    for (int i = 0; i < contours.size(); i++) {
        cv::Rect rect = boundingRect(contours[i]);
        approxPolyDP(contours[i], approxCurve, arcLength(Mat(contours[i]), true) * 0.03, true); //estimate the shape of contour
        double k = (rect.height + 0.0) / rect.width;
        if (progress) {
            drawContours(src, contours, i, Scalar(0, 255, 255));
            cv::imshow("tmp contour", src);
            cv::waitKey(0);
        }

        if (0.3 < k && k < 1.6 && rect.area() > 5000 && judgeIfRect(approxCurve, contours[i])) {
            cv::vector<int> tmpinnerIndexRecord = {};
            if (hierarchy[i][2] != -1 && judgeIfOutZYJBoard(contours, hierarchy, i, tmpinnerIndexRecord)) {
                // this is the zyjboard
                innerIndexRecord.swap(tmpinnerIndexRecord);
                MarkerContour.push_back(approxCurve[0]);
                MarkerContour.push_back(approxCurve[1]);
                MarkerContour.push_back(approxCurve[2]);
                MarkerContour.push_back(approxCurve[3]);
                drawContours(src, contours, i, Scalar(0, 0, 255));
            }
        }
        approxCurve.clear();
        //std::vector<cv::Point>(approxCurve).swap(approxCurve);
    }
    if (innerIndexRecord.size() == 0) {
        static int index_notRecog = 0;
        cv::imwrite("notRecog/notRecog" + std::to_string(index_notRecog) + ".png", src);
        index_notRecog++;
    }
    //2. get the inner contours
    cv::vector<cv::Point> contour_rect = {};    //inner rectangle
    cv::vector<cv::Point> contour_tria = {};    //inner triangle
    cv::vector<cv::Point> contour_cir = {};     //inner circle

    cv::vector<cv::Point> in_approxCurve = {};
    for (int index : innerIndexRecord) {
        Rect in_rect = boundingRect(contours[index]);

        approxPolyDP(contours[index], in_approxCurve, arcLength(Mat(contours[index]), true) * 0.045, true); //estimate the shape of contour
        if (in_approxCurve.size() == 4) {
            contour_rect = contours[index];
            drawContours(src, contours, index, Scalar(255, 0, 0));
        }
        else if (in_approxCurve.size() == 3) {
            contour_tria = contours[index];
            drawContours(src, contours, index, Scalar(0, 255, 0));
        }
        else {
            contour_cir = contours[index];
            drawContours(src, contours, index, Scalar(0, 0, 255));
        }
        if (progress) {
            cv::imshow("bound", src);
            cv::waitKey(0);
        }

        in_approxCurve.clear();
    }

    if (contour_rect.size() == 0 || contour_tria.size() == 0 || contour_cir.size() == 0)
        return false;

    //3. get the sorted corner of the board
    Point2d corner_Rect = getTheNearestCorner(MarkerContour, contour_rect[0]);
    Point2d corner_Tria = getTheNearestCorner(MarkerContour, contour_tria[0]);
    Point2d corner_Cir = getTheNearestCorner(MarkerContour, contour_cir[0]);
    Point2d corner_Last;
    for (int i = 0; i < MarkerContour.size(); i++) {
        auto cContour = MarkerContour[i];
        if (cContour != corner_Rect && cContour != corner_Tria && cContour != corner_Cir) {
            corner_Last = cContour;
        }
    }
    sortedCorners.push_back(corner_Rect);
    sortedCorners.push_back(corner_Tria);
    sortedCorners.push_back(corner_Last);
    sortedCorners.push_back(corner_Cir);
    if (progress) {
        cv::circle(src, sortedCorners[0], 5, Scalar(255, 0, 0), 2);
        cv::circle(src, sortedCorners[1], 5, Scalar(0, 255, 0), 2);
        cv::circle(src, sortedCorners[2], 5, Scalar(0, 0, 255), 2);
        cv::circle(src, sortedCorners[3], 5, Scalar(255, 0, 255), 2);
        cv::imshow("contours", src);
        cv::waitKey(0);
    }

    return true;
}

void ARCore::setZYJBoard(double length) {
    //1. set the zyjBoardInfo
    double zyjBoardLength = length;  //103mm
    zyjBoardCorner3dCoords.push_back({ 0.0, 0.0, 0.0 });                            //(0,0,0)
    zyjBoardCorner3dCoords.push_back({ zyjBoardLength, 0.0, 0.0 });                   //(1,0,0)
    zyjBoardCorner3dCoords.push_back({ zyjBoardLength, zyjBoardLength, 0.0 });      //(1,1,0)
    zyjBoardCorner3dCoords.push_back({ 0.0, zyjBoardLength, 0.0 });                 //(0,1,0)
}

/////////////////////////////////////////
// description: undistort the image considering camera distortion
// params - frame: the frame to distort
/////////////////////////////////////////
cv::Mat ARCore::undistort(const cv::Mat & frame) {
    cv::Mat result;
    cv::Mat mapx = cv::Mat(frame.size(), CV_32FC1);
    cv::Mat mapy = cv::Mat(frame.size(), CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
    cv::remap(frame, result, mapx, mapy, INTER_LINEAR);

    return result;
}

/////////////////////////////////////////
// description: undistort the points considering camera distortion
// params - points: the points to distort
/////////////////////////////////////////
cv::vector<cv::Point2d> ARCore::undistort(const cv::vector<cv::Point2d> & points) {
    cv::vector<Point2d> scene_corners = {};
    undistortPoints(points, scene_corners, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

    return scene_corners;
}

/////////////////////////////////////////
// description: use VTK as the render engine, init the VTK here
// notice: this function could be called by init()
// param: progress - if show the vtk window
/////////////////////////////////////////
void ARCore::initVTK(cv::Size windowSize, bool showVTK) {
    render->SetBackground(0, 0, 0);
    aCamera->SetViewUp(0, 0, -1);
    aCamera->SetPosition(0, 5, 0);
    aCamera->SetFocalPoint(0, 0, 0);
    aCamera->ComputeViewPlaneNormal();
    render->SetActiveCamera(aCamera);
    renWin->SetSize(windowSize.width, windowSize.height);
    renWin->AddRenderer(render);
    if (!showVTK) {
        renWin->SetOffScreenRendering(1);
    }
    renWin->Render();
    std::cout << "Position: " << render->GetActiveCamera()->GetPosition()[0] << " - " << render->GetActiveCamera()->GetPosition()[1] << " - " << render->GetActiveCamera()->GetPosition()[2] << std::endl;
    std::cout << "Focus: " << render->GetActiveCamera()->GetFocalPoint()[0] << " - " << render->GetActiveCamera()->GetFocalPoint()[1] << " - " << render->GetActiveCamera()->GetFocalPoint()[3] << std::endl;
    std::cout << "ViewUp: " << render->GetActiveCamera()->GetViewUp()[0] << " - " << render->GetActiveCamera()->GetViewUp()[1] << " - " << render->GetActiveCamera()->GetViewUp()[2] << std::endl;
}

/////////////////////////////////////////
// description: set VTK Camera information
// params - camPose: the camera pose
//          dx: CCD pixel-x/mm
//          dy: CCD pixel-y/mm
/////////////////////////////////////////
void ARCore::setVTKCamera(const CameraPose & camPose, double dx, double dy) {
    std::cout << "camera position   x = " << camPose.tvec.at<double>(0, 0) << "  y = " << camPose.tvec.at<double>(1, 0) << "  z = " << camPose.tvec.at<double>(2, 0) << std::endl;
    //1. position
    aCamera->SetPosition(camPose.tvec.at<double>(0, 0), camPose.tvec.at<double>(1, 0), camPose.tvec.at<double>(2, 0));

    //2. focus
    //std::cout << "camera inner matrix   fx = " << cameraMatrix.at<double>(0, 0) * dx << "  fy = " << cameraMatrix.at<double>(1, 1) * dy << std::endl;
    cv::Mat pos_focusInCamera = cv::Mat::zeros(cv::Size(1, 4), CV_64FC1);
    cv::Mat pos_Focus = cv::Mat::zeros(cv::Size(1, 4), CV_64FC1);
    pos_focusInCamera.at<double>(0, 0) = 0.0;
    pos_focusInCamera.at<double>(1, 0) = 0.0;
    pos_focusInCamera.at<double>(2, 0) = cameraMatrix.at<double>(0, 0) * dx;
    pos_focusInCamera.at<double>(3, 0) = 1.0;
    cv::Mat transformMat = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
    transformMat.at<double>(0, 0) = camPose.rotationMatrix.at<double>(0, 0);
    transformMat.at<double>(0, 1) = camPose.rotationMatrix.at<double>(0, 1);
    transformMat.at<double>(0, 2) = camPose.rotationMatrix.at<double>(0, 2);
    transformMat.at<double>(0, 3) = camPose.tvec.at<double>(0, 0);

    transformMat.at<double>(1, 0) = camPose.rotationMatrix.at<double>(1, 0);
    transformMat.at<double>(1, 1) = camPose.rotationMatrix.at<double>(1, 1);
    transformMat.at<double>(1, 2) = camPose.rotationMatrix.at<double>(1, 2);
    transformMat.at<double>(1, 3) = camPose.tvec.at<double>(1, 0);

    transformMat.at<double>(2, 0) = camPose.rotationMatrix.at<double>(2, 0);
    transformMat.at<double>(2, 1) = camPose.rotationMatrix.at<double>(2, 1);
    transformMat.at<double>(2, 2) = camPose.rotationMatrix.at<double>(2, 2);
    transformMat.at<double>(2, 3) = camPose.tvec.at<double>(2, 0);

    transformMat.at<double>(3, 0) = 0.0;
    transformMat.at<double>(3, 1) = 0.0;
    transformMat.at<double>(3, 2) = 0.0;
    transformMat.at<double>(3, 3) = 1.0;

    pos_Focus = transformMat * pos_focusInCamera;
    std::cout << "camera focus   x = " << pos_Focus.at<double>(0, 0) << "  y = " << pos_Focus.at<double>(1, 0) << " z = " << pos_Focus.at<double>(2, 0) << std::endl;
    aCamera->SetFocalPoint(pos_Focus.at<double>(0, 0), pos_Focus.at<double>(1, 0), pos_Focus.at<double>(2, 0));

    aCamera->ComputeViewPlaneNormal();

    //3. viewUp
    cv::Mat viewUpInCamera = cv::Mat::zeros(cv::Size(1, 3), CV_64FC1);
    viewUpInCamera.at<double>(0, 0) = 0.0;
    viewUpInCamera.at<double>(1, 0) = 0.0;
    viewUpInCamera.at<double>(2, 0) = 0.0;
    cv::Mat viewUp = cv::Mat::zeros(cv::Size(1, 3), CV_64FC1);
    viewUp = camPose.rotationMatrix * viewUpInCamera;
    aCamera->SetViewUp(viewUp.at<double>(0, 0), viewUp.at<double>(1, 0), viewUp.at<double>(2, 0));
    std::cout << "camera viewup   x = " << viewUp.at<double>(0, 0) << "  y = " << viewUp.at<double>(1, 0) << " z = " << viewUp.at<double>(2, 0) << std::endl;

    //4. view angle
    aCamera->SetViewAngle(65);
    aCamera->SetClippingRange(0.0475, 4502.3786);

    //aCamera->ParallelProjectionOff();
    render->SetActiveCamera(aCamera);
    render->ResetCamera();
    renWin->Render();
}

void ARCore::setVTKWindowSize(const cv::Mat & frame) {
    renWin->SetSize(frame.size().width, frame.size().height);
}

void ARCore::addCone2VTK(double height, double radius, int resolution) {
    vtkConeSource *cone = vtkConeSource::New();
    cone->SetHeight(height);
    cone->SetRadius(radius);
    cone->SetResolution(resolution);
    vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
    coneMapper->SetInputConnection(cone->GetOutputPort());
    vtkActor *coneActor = vtkActor::New();
    coneActor->SetMapper(coneMapper);
    render->AddActor(coneActor);
    coneActor->SetPosition(0, 0, 0);
    render->ResetCamera();
    renWin->Render();
}

void ARCore::addSpongeBob2VTK() {
    //1. read obj
    std::string filename = "spongebob.obj";
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    //2. read texture
    vtkSmartPointer<vtkPNGReader>bmpReader = vtkSmartPointer<vtkPNGReader>::New();
    bmpReader->SetFileName("spongebob.png");
    vtkSmartPointer<vtkTexture>texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(bmpReader->GetOutputPort());
    texture->InterpolateOn();

    vtkSmartPointer<vtkActor> actor =
        vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->SetTexture(texture);
    actor->SetScale(300);
    actor->SetPosition(0, 0, -150);
    //actor->RotateZ(180);
    //actor->RotateX(180);
    actor->RotateY(180);

    render->AddActor(actor);
}

bool ARCore::combineVTK2Frame(cv::Mat & frame, const CameraPose & camPose) {
    cv::Mat vtkFrame, vtkFrameClone;
    getFrameFromVTK(vtkFrame, false);
    vtkFrameClone = vtkFrame.clone();

    assert(vtkFrameClone.size() == frame.size());

    vector<Vec3d> originalPoint = { { 0,0,0 } };
    vector<Point2d> projectedPoints;
    projectPoints(originalPoint, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoints);
    Point2d newOriginalPoint = projectedPoints[0];
    int dx = frame.size().width / 2 - newOriginalPoint.x;
    int dy = frame.size().height / 2 - newOriginalPoint.y;

    if (vtkFrameClone.size() == frame.size()) {
        for (int row = 0; row < frame.rows; row++) {
            for (int col = 0; col < frame.cols; col++) {
                cv::Vec3b vtkColor = { 0, 0, 0 };
                vtkColor[0] = vtkFrameClone.at<cv::Vec3b>(row, col)[0];
                vtkColor[1] = vtkFrameClone.at<cv::Vec3b>(row, col)[1];
                vtkColor[2] = vtkFrameClone.at<cv::Vec3b>(row, col)[2];
                if (vtkColor[0] != 0 && vtkColor[1] != 0 && vtkColor[2] != 0) {
                    int newRow = row - dy < frame.rows ? (row - dy < 0 ? 0 : row - dy) : frame.rows - 1;
                    int newCol = col - dx < frame.cols ? ((col - dx < 0) ? 0 : col - dx) : frame.cols - 1;
                    frame.at<cv::Vec3b>(newRow, newCol)[0] = vtkColor[0];
                    frame.at<cv::Vec3b>(newRow, newCol)[1] = vtkColor[1];
                    frame.at<cv::Vec3b>(newRow, newCol)[2] = vtkColor[2];
                }
            }
        }
        return true;
    }

    return false;
}

/////////////////////////////////////////
// description: use VTK as the render engine, get the render result
// params - camPose: the camera pose
// return - vtkImg: VTK render result
/////////////////////////////////////////
void ARCore::getFrameFromVTK(cv::Mat & vtkImg, bool progress) {
    renWin->Render();
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
        vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renWin);
    windowToImageFilter->Update();
    vtkSmartPointer<vtkImageWriter> imgWriter = vtkSmartPointer<vtkImageWriter>::New();
    imgWriter->SetInputConnection(windowToImageFilter->GetOutputPort());
    vtkImageData *imgRendered = imgWriter->GetImageDataInput(0);
    vtkImage2Mat(imgRendered, vtkImg);
    cv::flip(vtkImg, vtkImg, 0);
    if (progress) {
        cv::imshow("vtk frame", vtkImg);
        cv::waitKey(0);
    }
}

/////////////////////////////////////////
// description: draw the cube based on the recogonized ZYJBoard
// params - frame: the original frame with zyjboard
//          boardLength: the length(mm) of the zyjboard
//          camPose: the camera pose
/////////////////////////////////////////
void ARCore::drawZYJBoardCube(cv::Mat & frame, double boardLength, const CameraPose & camPose) {
    vector<Vec3d> _cube3dPoints = {};

    assert(boardLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    _cube3dPoints.push_back({ 0.0, 0.0, 0.0 });                         // (0,0,0)
    _cube3dPoints.push_back({ boardLength, 0.0, 0.0 });                 // (1,0,0)
    _cube3dPoints.push_back({ boardLength, boardLength, 0.0 });         // (1,1,0)
    _cube3dPoints.push_back({ 0.0, boardLength, 0.0 });                 // (0,1,0)

    _cube3dPoints.push_back({ 0.0, 0.0, -boardLength });                 // (0,0,1)
    _cube3dPoints.push_back({ boardLength, 0.0, -boardLength });         // (1,0,1)
    _cube3dPoints.push_back({ boardLength, boardLength, -boardLength }); // (1,1,1)
    _cube3dPoints.push_back({ 0.0, boardLength, -boardLength });         // (0,1,1)

    vector<Point2d> projectedPoints;
    projectPoints(_cube3dPoints, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoints);

    for (int i = 0; i<4; i++) {
        if (i <= 2) line(frame, projectedPoints[i], projectedPoints[i + 1], Scalar(255, 255, 0), 1, 4);
        if (i>2) line(frame, projectedPoints[i], projectedPoints[0], Scalar(255, 255, 0), 1, 4);
    }

    for (int i = 0, j = 4; i<4; i++, j++) {
        line(frame, projectedPoints[i], projectedPoints[j], Scalar(0, 0, 255), 1, 4);
    }

    for (int i = 4; i<8; i++) {
        if (i <= 6) line(frame, projectedPoints[i], projectedPoints[i + 1], Scalar(0, 255, 0), 1, 4);
        if (i>6) line(frame, projectedPoints[i], projectedPoints[4], Scalar(0, 255, 0), 1, 4);
    }

}

void ARCore::drawPoint(cv::Mat & frame, cv::Point3d original, const CameraPose & camPose, cv::Scalar pointColor, int thickness) {
    vector<Vec3d> _Point = { original };
    vector<Point2d> projectedPoint;
    projectPoints(_Point, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoint);
    //projectedPoint = undistort(projectedPoint);
    cv::circle(frame, projectedPoint[0], thickness, pointColor, -1);
}

void ARCore::drawPolyLine(cv::Mat & frame, vector<Vec3d> polyPoints, bool closed, const CameraPose & camPose, cv::Scalar polyColor, int thickness) {

    assert(polyPoints.size() > 0);

    std::vector<Point2d> projectedPoints = {};
    projectPoints(polyPoints, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoints);
    //projectedPoints = undistort(projectedPoints);

    //collect points
    Point points[1][20000];
    for (int i = 0; i < projectedPoints.size(); i++) {
        points[0][i] = projectedPoints[i];
    }
    const Point* pt[1] = { points[0] };
    int npt[1] = { projectedPoints.size() };

    //draw the poly line 
    cv::polylines(frame, pt, npt, 1, closed, polyColor, thickness);
}

void ARCore::drawPlane(cv::Mat & frame, vector<Vec3d> planeCorners, const CameraPose & camPose, cv::Scalar planeColor, cv::Scalar borderColor, int thickness) {
    assert(planeCorners.size() > 2);

    std::vector<Point2d> projectedPoints = {};
    projectPoints(planeCorners, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoints);
    //projectedPoints = undistort(projectedPoints);

    //collect points
    Point points[1][20000];
    for (int i = 0; i < projectedPoints.size(); i++) {
        points[0][i] = projectedPoints[i];
    }
    const Point* pt[1] = { points[0] };
    int npt[1] = { projectedPoints.size() };

    cv::fillPoly(frame, pt, npt, 1, planeColor, 8);

    drawPolyLine(frame, planeCorners, true, camPose, borderColor, thickness);
}

void ARCore::drawCubeLine(cv::Mat & frame, cv::Point3d original, double width, double length, double height, const CameraPose & camPose, cv::Scalar lineColor, int thickness) {
    vector<Vec3d> _cube3dPoints = {};

    // set coordinate system in the middle of the marker, with Z pointing out
    _cube3dPoints.push_back({ original.x, original.y, original.z });                         // (0,0,0)
    _cube3dPoints.push_back({ original.x + width, original.y, original.z });                 // (1,0,0)
    _cube3dPoints.push_back({ original.x + width, original.y + length, original.z });         // (1,1,0)
    _cube3dPoints.push_back({ original.x, original.y + length, original.z });                 // (0,1,0)

    _cube3dPoints.push_back({ original.x, original.y, original.z + height });                 // (0,0,1)
    _cube3dPoints.push_back({ original.x + width, original.y, original.z + height });         // (1,0,1)
    _cube3dPoints.push_back({ original.x + width, original.y + length, original.z + height }); // (1,1,1)
    _cube3dPoints.push_back({ original.x, original.y + length, original.z + height });         // (0,1,1)

    vector<Point2d> projectedPoints;
    projectPoints(_cube3dPoints, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoints);
    //projectedPoints = undistort(projectedPoints);

    for (int i = 0; i<4; i++) {
        if (i <= 2) line(frame, projectedPoints[i], projectedPoints[i + 1], lineColor, thickness, 4);
        if (i>2) line(frame, projectedPoints[i], projectedPoints[0], lineColor, thickness, 4);
    }

    for (int i = 0, j = 4; i<4; i++, j++) {
        line(frame, projectedPoints[i], projectedPoints[j], lineColor, thickness, 4);
    }

    for (int i = 4; i<8; i++) {
        if (i <= 6) line(frame, projectedPoints[i], projectedPoints[i + 1], lineColor, thickness, 4);
        if (i>6) line(frame, projectedPoints[i], projectedPoints[4], lineColor, thickness, 4);
    }
}

void ARCore::drawCube(cv::Mat & frame, cv::Point3d original, double width, double length, double height, const CameraPose & camPose, cv::Scalar cubeColor) {
    vector<Vec3d> _cube3dPoints = {};

    // set coordinate system in the middle of the marker, with Z pointing out
    _cube3dPoints.push_back({ original.x, original.y, original.z });                         // (0,0,0)
    _cube3dPoints.push_back({ original.x + width, original.y, original.z });                 // (1,0,0)
    _cube3dPoints.push_back({ original.x + width, original.y + length, original.z });         // (1,1,0)
    _cube3dPoints.push_back({ original.x, original.y + length, original.z });                 // (0,1,0)

    _cube3dPoints.push_back({ original.x, original.y, original.z + height });                 // (0,0,1)
    _cube3dPoints.push_back({ original.x + width, original.y, original.z + height });         // (1,0,1)
    _cube3dPoints.push_back({ original.x + width, original.y + length, original.z + height }); // (1,1,1)
    _cube3dPoints.push_back({ original.x, original.y + length, original.z + height });         // (0,1,1)

    vector<Point2d> projectedPoints;
    projectPoints(_cube3dPoints, camPose.rvec, camPose.tvec, cameraMatrix, distCoeffs, projectedPoints);
    //projectedPoints = undistort(projectedPoints);

    if (!(projectedPoints[0].x <= 1000000 && projectedPoints[0].y <= 1000000
        && projectedPoints[1].x <= 1000000 && projectedPoints[1].y <= 1000000
        && projectedPoints[2].x <= 1000000 && projectedPoints[2].y <= 1000000
        && projectedPoints[3].x <= 1000000 && projectedPoints[3].y <= 1000000
        && projectedPoints[4].x <= 1000000 && projectedPoints[4].y <= 1000000
        && projectedPoints[5].x <= 1000000 && projectedPoints[5].y <= 1000000
        && projectedPoints[6].x <= 1000000 && projectedPoints[6].y <= 1000000
        && projectedPoints[7].x <= 1000000 && projectedPoints[7].y <= 1000000))
        return;

    //1. draw bottom plate
    {
        Point bottomPoints[1][4];
        bottomPoints[0][0] = projectedPoints[0];
        bottomPoints[0][1] = projectedPoints[1];
        bottomPoints[0][2] = projectedPoints[2];
        bottomPoints[0][3] = projectedPoints[3];
        const Point* pt[1] = { bottomPoints[0] };
        int npt[1] = { 4 };

        cv::fillPoly(frame, pt, npt, 1, cubeColor, 8);
    }

    //2. draw left plate
    {
        Point leftPoints[1][4];
        leftPoints[0][0] = projectedPoints[0];
        leftPoints[0][1] = projectedPoints[4];
        leftPoints[0][2] = projectedPoints[7];
        leftPoints[0][3] = projectedPoints[3];
        const Point* pt[1] = { leftPoints[0] };
        int npt[1] = { 4 };

        cv::fillPoly(frame, pt, npt, 1, cubeColor, 8);
    }

    //3. draw back plate
    {
        Point backPoints[1][4];
        backPoints[0][0] = projectedPoints[0];
        backPoints[0][1] = projectedPoints[4];
        backPoints[0][2] = projectedPoints[5];
        backPoints[0][3] = projectedPoints[1];
        const Point* pt[1] = { backPoints[0] };
        int npt[1] = { 4 };

        cv::fillPoly(frame, pt, npt, 1, cubeColor, 8);
    }

    //4. draw right plate
    {
        Point rightPoints[1][4];
        rightPoints[0][0] = projectedPoints[1];
        rightPoints[0][1] = projectedPoints[5];
        rightPoints[0][2] = projectedPoints[6];
        rightPoints[0][3] = projectedPoints[2];
        const Point* pt[1] = { rightPoints[0] };
        int npt[1] = { 4 };

        cv::fillPoly(frame, pt, npt, 1, cubeColor, 8);
    }

    //5. draw front plate
    {
        Point frontPoints[1][4];
        frontPoints[0][0] = projectedPoints[2];
        frontPoints[0][1] = projectedPoints[6];
        frontPoints[0][2] = projectedPoints[7];
        frontPoints[0][3] = projectedPoints[3];
        const Point* pt[1] = { frontPoints[0] };
        int npt[1] = { 4 };

        cv::fillPoly(frame, pt, npt, 1, cubeColor, 8);
    }

    //6. draw top plate
    {
        Point topPoints[1][4];
        topPoints[0][0] = projectedPoints[4];
        topPoints[0][1] = projectedPoints[5];
        topPoints[0][2] = projectedPoints[6];
        topPoints[0][3] = projectedPoints[7];
        const Point* pt[1] = { topPoints[0] };
        int npt[1] = { 4 };

        cv::fillPoly(frame, pt, npt, 1, cubeColor, 8);
    }

    //7. draw border
    for (int i = 0; i<4; i++) {
        if (i <= 2) line(frame, projectedPoints[i], projectedPoints[i + 1], { 255,255,255 }, 1, 1);
        if (i>2) line(frame, projectedPoints[i], projectedPoints[0], { 255,255,255 }, 1, 1);
    }

    for (int i = 0, j = 4; i<4; i++, j++) {
        line(frame, projectedPoints[i], projectedPoints[j], { 255,255,255 }, 1, 1);
    }

    for (int i = 4; i<8; i++) {
        if (i <= 6) line(frame, projectedPoints[i], projectedPoints[i + 1], { 255,255,255 }, 1, 1);
        if (i>6) line(frame, projectedPoints[i], projectedPoints[4], { 255,255,255 }, 1, 1);
    }
}

void ARCore::drawCoordinate(cv::Mat & frame, const CameraPose & camPose) {
    //1. draw axis
    std::vector<cv::Vec3d> xLine = { { 0,0,0 },{ 50, 0, 0 } };
    std::vector<cv::Vec3d> yLine = { { 0,0,0 },{ 0, 50, 0 } };
    std::vector<cv::Vec3d> zLine = { { 0,0,0 },{ 0, 0, 50 } };
    drawPolyLine(frame, xLine, false, camPose, { 255,0,0 }, 1);
    drawPolyLine(frame, yLine, false, camPose, { 0,255,0 }, 1);
    drawPolyLine(frame, zLine, false, camPose, { 0,0,255 }, 1);
    //2. draw arrow
    std::vector<cv::Vec3d> xArrow = { { 50,0,0 },{ 45,5,0 },{ 50,0,0 },{ 45,-5,0 } };
    std::vector<cv::Vec3d> yArrow = { { 0,50,0 },{ 5,45,0 },{ 0,50,0 },{ -5,45,0 } };
    std::vector<cv::Vec3d> zArrow = { { 0,0,50 },{ 5,5,45 },{ 0,0,50 },{ -5,5,45 } };
    drawPolyLine(frame, xArrow, false, camPose, { 255,0,0 }, 1);
    drawPolyLine(frame, yArrow, false, camPose, { 0,255,0 }, 1);
    drawPolyLine(frame, zArrow, false, camPose, { 0,0,255 }, 1);
}

bool ARCore::judgeIfRect(vector<Point> corners, vector<Point2i> contour) {
    if (corners.size() != 4)
        return false;

    int row_leftTop = 0, col_leftTop = 0, row_rightBottom = 0, col_rightBottom = 0;
    for (Point2d tmp : corners) {
        if (row_leftTop == 0 && col_leftTop == 0 && row_rightBottom == 0 && col_rightBottom == 0) {
            row_leftTop = tmp.y;
            col_leftTop = tmp.x;
            row_rightBottom = tmp.y;
            col_rightBottom = tmp.x;
        }
        else {
            if (tmp.x < col_leftTop)
                col_leftTop = tmp.x;
            if (tmp.x > col_rightBottom)
                col_rightBottom = tmp.x;
            if (tmp.y < row_leftTop)
                row_leftTop = tmp.y;
            if (tmp.y > row_rightBottom)
                row_rightBottom = tmp.y;
        }
    }

    double areaSize = contourArea(contour, false);
    int area = (row_rightBottom - row_leftTop) * (col_rightBottom - col_leftTop);
    //std::cout << "areaSize: " << areaSize << "area real: " << area << std::endl;
    if ((2 * areaSize - area) < 0)
        return false;
    return true;
}

bool ARCore::judgeIfOutZYJBoard(const std::vector<std::vector<cv::Point>> & contours, const vector<Vec4i> & hierarchy, int index, std::vector<int> & innerIndexRecord) {
    Vec4i c_hier = hierarchy[index];
    int innerNum = 0;
    for (int i = 0; i < hierarchy.size(); i++) {
        auto tmp_hier = hierarchy[i];
        if (tmp_hier[3] == index) {
            if (contours[i].size() >= 15) {
                innerIndexRecord.push_back(i);
                innerNum++;
            }
        }
    }
    if (innerNum == 3)
        return true;
    else
        return false;
}

bool isTmpPosValueSmaller(TmpPosValue t1, TmpPosValue t2)
{
    return t1.value < t2.value;
}
Point ARCore::getTheNearestCorner(vector<Point2d> MarkerContour, Point2d point) {
    auto dist1 = sqrt((MarkerContour[0].x - point.x) * (MarkerContour[0].x - point.x) + (MarkerContour[0].y - point.y) * (MarkerContour[0].y - point.y));
    auto dist2 = sqrt((MarkerContour[1].x - point.x) * (MarkerContour[1].x - point.x) + (MarkerContour[1].y - point.y) * (MarkerContour[1].y - point.y));
    auto dist3 = sqrt((MarkerContour[2].x - point.x) * (MarkerContour[2].x - point.x) + (MarkerContour[2].y - point.y) * (MarkerContour[2].y - point.y));
    auto dist4 = sqrt((MarkerContour[3].x - point.x) * (MarkerContour[3].x - point.x) + (MarkerContour[3].y - point.y) * (MarkerContour[3].y - point.y));

    TmpPosValue tpv_d1, tpv_d2, tpv_d3, tpv_d4;
    tpv_d1.index = 0;
    tpv_d1.value = dist1;
    tpv_d2.index = 1;
    tpv_d2.value = dist2;
    tpv_d3.index = 2;
    tpv_d3.value = dist3;
    tpv_d4.index = 3;
    tpv_d4.value = dist4;

    vector<TmpPosValue> vec_dist = { tpv_d1, tpv_d2, tpv_d3, tpv_d4 };
    sort(vec_dist.begin(), vec_dist.end(), isTmpPosValueSmaller);

    return MarkerContour[vec_dist[0].index];
}

void ARCore::contoursMat2Vec(const std::vector<cv::Mat> & contoursMat, std::vector<std::vector<cv::Point>> & contours, const cv::Mat & hierarchyMat, std::vector<cv::Vec4i> & hierarchy) {
    if (contoursMat.size() == 0)
        return;
    for (int itemIndex = 0; itemIndex < contoursMat.size(); itemIndex++) {
        std::vector<cv::Point> contourItem = {};

        for (int row = 0; row < contoursMat[itemIndex].rows; row++) {
            //1. contour item
            cv::Point pointItem = { contoursMat[itemIndex].at<int>(row, 0),contoursMat[itemIndex].at<int>(row, 1) };
            contourItem.push_back(pointItem);
        }
        contours.push_back(contourItem);

        //2. hierarchy item
        hierarchy.push_back(hierarchyMat.at<cv::Vec4i>(0, itemIndex));
    }
}

void ARCore::saveCalibrationResult() {
    //1. cameraMatrix
    std::ofstream ofs1("cameraMatrix.bin", std::ios::out | std::ios::binary);
    boost::archive::binary_oarchive oa1(ofs1);
    oa1 << cameraMatrix;
    //2. distCoeffs
    std::ofstream ofs2("distCoeffs.bin", std::ios::out | std::ios::binary);
    boost::archive::binary_oarchive oa2(ofs2);
    oa2 << distCoeffs;

    ofs1.close();
    ofs2.close();
}

void ARCore::readCalibrationResult() {
    //1. cameraMatrix
    ifstream inf1("cameraMatrix.bin", std::ios::binary);
    boost::archive::binary_iarchive ia1(inf1);
    ia1 >> cameraMatrix;
    //2. distCoeffs
    ifstream inf2("distCoeffs.bin", std::ios::binary);
    boost::archive::binary_iarchive ia2(inf2);
    ia2 >> distCoeffs;

    inf1.close();
    inf2.close();
}

void ARCore::vtkImage2Mat(vtkImageData *image, cv::Mat & vtkImg)
{
    int dim[3];
    image->GetDimensions(dim);
    int imgType = CV_8UC1;
    if (image->GetNumberOfScalarComponents() == 1)
        imgType = CV_8UC1;
    if (image->GetNumberOfScalarComponents() == 3)
        imgType = CV_8UC3;

    cv::Mat tmp(dim[1], dim[0], imgType, image->GetScalarPointer());
    cv::cvtColor(tmp, vtkImg, CV_BGR2RGB);
}