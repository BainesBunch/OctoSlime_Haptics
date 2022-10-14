#include <Arduino.h>
#include "globals.h"
#include <GyverOLED.h>

namespace UI
{

    void Setup();
    void DrawSplash();
    void MainUIFrame();
    void SetIMUStatus(uint8 imuID, bool Status);
    void DrawFrame(void);
    void SetImuCount(uint8 IMUs);
    void SetMessage(uint8 MessageID, ...);
    void setBatStatus(float level);
    void DrawCalibrationScreen(uint8_t IMUID);
    void DrawCalibrationContdown(uint8_t Count);
    void DrawCalibrationAborted();
    void DrawCalibrationInstructions();
    void DrawCalibrationProgress(float TotalSampleCount, float SampleCount);
    void DrawCalibrationComplete();
    void DrawCalibrationAdvice(uint8 imuID);
}