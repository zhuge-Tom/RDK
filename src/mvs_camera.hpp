#pragma once
#include <MvCameraControl.h>
#include <cstdint>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace rmcs::mvs {
void check_mvs(const char *action, int code) {
    if (code != MV_OK) {
        throw std::runtime_error(std::string(action) + " failed, MVS code=" +
                                 std::to_string(static_cast<std::uint32_t>(code)));
    }
}

class Camera {
  public:
    Camera(float exposure, float gain, float frame_rate) {
        auto devices = MV_CC_DEVICE_INFO_LIST{};
        check_mvs("enumerate devices", MV_CC_EnumDevices(MV_USB_DEVICE, &devices));
        if (devices.nDeviceNum == 0 || devices.pDeviceInfo[0] == nullptr) {
            throw std::runtime_error("no Hikrobot USB camera found");
        }
        check_mvs("create handle", MV_CC_CreateHandle(&handle_, devices.pDeviceInfo[0]));
        check_mvs("open device", MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0));
        check_mvs("disable auto exposure",
                  MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off"));
        check_mvs("set exposure", MV_CC_SetFloatValue(handle_, "ExposureTime", exposure));
        check_mvs("disable auto gain", MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off"));
        check_mvs("set gain", MV_CC_SetFloatValue(handle_, "Gain", gain));
        check_mvs("disable trigger", MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off"));
        check_mvs("enable frame rate",
                  MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true));
        check_mvs("set frame rate",
                  MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate));
        check_mvs("start grabbing", MV_CC_StartGrabbing(handle_));
    }

    ~Camera() {
        if (handle_ == nullptr)
            return;
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
    }

    auto read(unsigned int timeout_ms = 1000) -> cv::Mat {
        auto frame = MV_FRAME_OUT{};
        if (MV_CC_GetImageBuffer(handle_, &frame, timeout_ms) != MV_OK)
            return {};
        auto raw = cv::Mat(static_cast<int>(frame.stFrameInfo.nHeight),
                           static_cast<int>(frame.stFrameInfo.nWidth), CV_8UC1, frame.pBufAddr);
        auto bgr = cv::Mat{};
        cv::cvtColor(raw, bgr, cv::COLOR_BayerBG2BGR);
        MV_CC_FreeImageBuffer(handle_, &frame);
        return bgr;
    }

  private:
    void *handle_ = nullptr;
};

} // namespace rmcs::mvs
