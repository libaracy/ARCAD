// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: reference additional headers your program requires here
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
////VTK
#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkProperty.h"
#include <vtkAutoInit.h>
#include "vtkInteractorStyleTrackballActor.h"
#include "vtkRenderWindow.h"  
#include "vtkRenderWindowInteractor.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
////BOOST
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <Windows.h>

using namespace cv;
using namespace std;