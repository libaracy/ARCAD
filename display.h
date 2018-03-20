#pragma once

#ifdef CORE_EXPORT
#define CORE_EXPORT_API __declspec(dllexport)
#else
#define CORE_EXPORT_API
#endif


using FramePtr = std::shared_ptr<cv::Mat>;

CORE_EXPORT_API void setDisplayCallback(std::function<void(FramePtr)> cb);

CORE_EXPORT_API void display();