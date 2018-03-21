#pragma once
#include "DrawableData.h"

#ifdef CORE_EXPORT
#define CORE_EXPORT_API __declspec(dllexport)
#else
#define CORE_EXPORT_API
#endif


using FramePtr = std::shared_ptr<cv::Mat>;

using DisplayData = std::vector<std::shared_ptr<const DrawableData>>;

CORE_EXPORT_API void setDisplayCallback(std::function<void(FramePtr)> cb);

CORE_EXPORT_API void setDisplayData(DisplayData&& data);

CORE_EXPORT_API void display();

CORE_EXPORT_API void toggleFloor();

CORE_EXPORT_API void toggleSpongeBob();
