#ifndef PXBLUEFOXCAMERA_H
#define PXBLUEFOXCAMERA_H

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include "PxCamera.h"

class PxBluefoxCamera : public PxCamera
{
public:
    explicit PxBluefoxCamera(mvIMPACT::acquire::Device* _dev);

    bool init(void);
    void destroy(void);

    bool setConfig(const PxCameraConfig& config, bool master = true);

    bool start(void);
    bool stop(void);

    bool grabFrame(cv::Mat& image, uint32_t& skippedFrames,
                   uint32_t& sequenceNum);

private:
    bool setSlave(void);
    bool setExternalTrigger(void);
    bool setFrameRate(float frameRate);
    bool setMode(PxCameraConfig::Mode mode);
    bool setExposureTime(uint32_t exposureTime);
    bool setGain(uint32_t gain);
    bool setGainDB(float gain_dB);
    bool setPixelClock(uint32_t pixelClockKHz);
    bool setDesiredAverageGreyValue(uint32_t greyValue);

    float getFramesPerSecond(void);

    int triggerPulseWidth(void) const;

    void imageHandler(void);

    bool convertToCvMat(const mvIMPACT::acquire::Request* request, cv::Mat& image);

    mvIMPACT::acquire::Device* dev;
    std::tr1::shared_ptr<mvIMPACT::acquire::FunctionInterface> functionInterface;
    std::tr1::shared_ptr<mvIMPACT::acquire::CameraSettingsBlueFOX> cameraSettings;
    std::tr1::shared_ptr<mvIMPACT::acquire::IOSubSystemBlueFOX> io;
    std::tr1::shared_ptr<mvIMPACT::acquire::Statistics> stats;

    Glib::Thread* imageThread;
    bool imageAvailable;
    Glib::Mutex imageMutex;
    std::tr1::shared_ptr<Glib::Cond> imageAvailableCond;
    bool exitImageThread;

    cv::Mat image;
    uint32_t imageSequenceNr;

    float frameRate;
    int timeout_ms;

    uint64_t serialNum;
    uint32_t lastSequenceNum;

    friend class PxBluefoxStereoCamera;
};

#endif
