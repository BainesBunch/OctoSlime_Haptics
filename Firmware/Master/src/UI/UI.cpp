#include <UI\UI.h>
namespace UI
{

    GyverOLED<SSD1306_128x64> oled;

    const unsigned char octopus[] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0,
        0x70, 0x18, 0x08, 0x08, 0x0C, 0x04, 0x04, 0x04, 0x04, 0xC4,
        0xC4, 0x0C, 0x0C, 0x08, 0x18, 0x10, 0x70, 0xE0, 0xC0, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
        0xC0, 0x40, 0x60, 0x60, 0x20, 0x20, 0x30, 0x10, 0x10, 0x10,
        0x10, 0x10, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
        0x10, 0x10, 0x10, 0x10, 0x30, 0x20, 0x20, 0x60, 0x40, 0xC0,
        0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xC0, 0xF0, 0x38, 0x0C, 0x06, 0x06, 0x03, 0x03, 0x61, 0x61,
        0xC1, 0x81, 0x81, 0x01, 0x01, 0x01, 0x01, 0x03, 0x83, 0xC6,
        0xEE, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06,
        0x06, 0x84, 0xE4, 0xFC, 0xFE, 0x7E, 0xFF, 0xFD, 0x3C, 0x10,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3, 0x7F, 0x00, 0x80,
        0xE0, 0x38, 0x1C, 0x06, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x90,
        0x90, 0x80, 0x00, 0x98, 0x98, 0x90, 0x00, 0x80, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x03, 0x07, 0x0E, 0x1C, 0xF0, 0xC0, 0x00, 0xFE, 0xFF, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x60, 0x30,
        0x10, 0x18, 0x88, 0x88, 0x08, 0x08, 0x0C, 0x08, 0x08, 0x08,
        0x18, 0x18, 0xB0, 0xE0, 0x00, 0x00, 0xF0, 0xFC, 0x3F, 0x7B,
        0x7B, 0x79, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xF8, 0xFF, 0x03, 0x00, 0x00, 0x1F, 0x7F, 0xE0,
        0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
        0x60, 0xE0, 0xA0, 0x00, 0x00, 0x03, 0x1B, 0x1F, 0x1F, 0x37,
        0x3C, 0x3F, 0x37, 0x1F, 0x1D, 0x09, 0x00, 0x00, 0x00, 0xA0,
        0xE0, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x80, 0xE0, 0x78, 0x1F, 0x00, 0x00, 0x7F, 0xF8, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xF8, 0xE8, 0xFF,
        0xFF, 0xF0, 0x80, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0x40, 0x40,
        0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xC0, 0xC0, 0x80, 0x80,
        0x00, 0x00, 0x00, 0x00, 0x1F, 0x79, 0xE0, 0x80, 0x80, 0x00,
        0xC1, 0xC1, 0xC3, 0xC2, 0x86, 0x8E, 0x8E, 0x9A, 0x92, 0x33,
        0x21, 0x61, 0x60, 0x40, 0x41, 0xCF, 0xDC, 0xF0, 0xE0, 0xC0,
        0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x03, 0x0E, 0x1C, 0x18, 0xB0, 0xA0, 0x21, 0x63, 0x67,
        0x6C, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x03, 0x02, 0x06, 0x04, 0x0C, 0x0C, 0x08, 0x08, 0x08, 0x08,
        0x08, 0x08, 0x0C, 0x0C, 0x04, 0x06, 0x03, 0x03, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x3C, 0x26, 0x23, 0x21,
        0x30, 0x30, 0x18, 0x18, 0x0E, 0x07, 0x00, 0x00, 0x00, 0x18,
        0x3C, 0xAC, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F,
        0x07, 0x0E, 0x1F, 0x11, 0x10, 0x30, 0x30, 0xB0, 0xF0, 0x70,
        0x10, 0x18, 0x08, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06,
        0xFC, 0xF0, 0x00, 0x00, 0x00, 0x81, 0xC1, 0xC3, 0x43, 0x67,
        0x27, 0x2D, 0x3F, 0x3B, 0x1F, 0x1E, 0x1F, 0x1F, 0x1E, 0x18,
        0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x10, 0x10, 0x10, 0x11,
        0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x31, 0x31, 0x31,
        0x11, 0x11, 0x11, 0x11, 0x10, 0x18, 0x18, 0x08, 0x08, 0x0C,
        0x00, 0x00, 0x00, 0xC0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x18, 0x30, 0x60, 0xC0, 0x81, 0x03, 0x02, 0x06,
        0x04, 0x0E, 0x0F, 0x0F, 0x1F, 0x1F, 0x1E, 0x1E, 0x1F, 0x1F,
        0x1F, 0x1F, 0x1F, 0x17, 0x17, 0x13, 0x10, 0x18, 0x18, 0x08,
        0x08, 0x0C, 0x04, 0x06, 0x03, 0x03, 0x01, 0x98, 0xDC, 0xD4,
        0xFC, 0xCC, 0x80, 0xCE, 0x6E, 0x6E, 0x3E, 0x1C, 0x0F, 0x01,
        0x00, 0x3E, 0x77, 0xC1, 0x80, 0x80, 0x00, 0x00, 0x18, 0x08,
        0x0C, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x06, 0x86, 0x8E, 0xFE,
        0x7E, 0x0E, 0x08, 0x0C, 0x0E, 0x0E, 0x1E, 0x1E, 0x1E, 0x18,
        0x1C, 0x1E, 0x1E, 0x1E, 0x3A, 0x3E, 0x3C, 0x38, 0x30, 0x3E,
        0x3A, 0x3E, 0x3E, 0xBE, 0xFE, 0xF8, 0x78, 0x38, 0x1C, 0x0E,
        0x03, 0x01, 0x80, 0xC0, 0xC0, 0x80, 0x80, 0xC0, 0xC0, 0xC0,
        0x07, 0xFF, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x7C, 0xE0, 0xC0, 0x80,
        0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x06, 0x06, 0x0E,
        0x0E, 0x1E, 0x1E, 0x3C, 0x30, 0x3C, 0x7C, 0x5C, 0x5C, 0xDC,
        0x9C, 0x90, 0x90, 0x10, 0x10, 0x98, 0xD8, 0x48, 0x68, 0x68,
        0x68, 0x6C, 0x6C, 0x44, 0xC4, 0x87, 0x07, 0x03, 0x03, 0x01,
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x02,
        0x02, 0x03, 0x03, 0x73, 0xFF, 0x8F, 0x07, 0x03, 0x03, 0x03,
        0x01, 0x01, 0x83, 0xC3, 0x66, 0x3C, 0x38, 0x30, 0x30, 0x30,
        0x10, 0x10, 0x10, 0x18, 0x08, 0x08, 0x8C, 0x84, 0xC6, 0x42,
        0xC3, 0xC1, 0x00, 0x00, 0xB0, 0xF0, 0xF8, 0xF8, 0xB8, 0xF0,
        0xF1, 0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0x0F, 0x07, 0x07, 0x03,
        0x03, 0x0F, 0x3F, 0xFF, 0xDC, 0x9C, 0xDC, 0xD8, 0xC0, 0xC0,
        0xC0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x39, 0x71, 0xC3,
        0x82, 0x06, 0x04, 0x8C, 0x88, 0x9E, 0x9E, 0x1E, 0x1E, 0x3E,
        0x3C, 0x30, 0x78, 0x78, 0x7C, 0x78, 0x78, 0xF8, 0xE0, 0xE0,
        0xE1, 0x81, 0x81, 0x8F, 0x8C, 0x80, 0xC0, 0xC0, 0x40, 0x40,
        0x60, 0x60, 0x30, 0x1F, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x02, 0x06, 0x06, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x05, 0x07, 0x07, 0x07, 0x03, 0x02,
        0x02, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x15, 0x31, 0x60,
        0x60, 0x40, 0xC0, 0xC0, 0x88, 0x8E, 0x86, 0x83, 0x81, 0x81,
        0x81, 0x81, 0xC1, 0x41, 0x41, 0x61, 0x33, 0x1E, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00};

    void Setup()
    {
        oled.init();
        oled.clear();
        oled.update();
    }

    void setBatStatus(float level)
    {
        oled.roundRect(0, 51, 127, 63, OLED_CLEAR);
        oled.setCursorXY(4, 50);
        oled.print(F("Bat"));
        oled.setCursorXY(23, 50);
        oled.print(uint8_t(level * 100));
        oled.setCursorXY(43, 50);
        oled.print(F("%"));
        oled.rect(52, 50, 122, 57, OLED_STROKE);
        oled.rect(52, 50, (level * 72) + 50, 57, OLED_FILL || OLED_STROKE);
        DrawFrame();
    }

    void DrawSplash()
    {

        oled.clear();
        oled.update();

        for (uint8 Y = 1; Y < 32; Y += 4)
        {
            ESP.wdtFeed();
            oled.roundRect((Y * 2), Y, 128 - (Y * 2), 64 - Y, OLED_STROKE);
            oled.update();
        }

        oled.clear();
        ESP.wdtFeed();
        oled.drawBitmap(0, 0, octopus, 128, 64, BITMAP_NORMAL, BUF_ADD);
        ESP.wdtFeed();
        oled.update();
    }

    void DrawCalibrationScreen(uint8_t IMUID)
    {
        uint8_t Node = IMUID % 8;
        char Bank = (IMUID < 8) ? 'A' : 'B';

        oled.clear();
        oled.setScale(1);
        oled.update();
        oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.setCursorXY(4, 4);
        oled.print(F("IMU Calibration Mode"));
        oled.setCursorXY(11, 16);
        oled.printf("Sensor %c on Node %u ", Bank, Node);
        oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.update();
    }

    void DrawCalibrationContdown(uint8_t Count)
    {
        oled.setScale(1);
        oled.update();
        oled.setCursorXY(12, 28);
        oled.print(F("Please flip Sensor"));
        oled.setCursorXY(30, 40);
        oled.print(F("In"));
        oled.roundRect(50, 40, 65, 46, OLED_CLEAR);
        oled.setCursorXY(50, 40);
        oled.print(Count);
        oled.setCursorXY(70, 40);
        oled.print(F("Seconds"));
        oled.setCursorXY(4, 53);
        oled.print(F("To Begin Calibration"));
        oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.update();
    }

    void DrawCalibrationProgress(float TotalSampleCount, float SampleCount)
    {
        float level = (SampleCount / TotalSampleCount);
        oled.roundRect(4, 50, 122, 57, OLED_CLEAR);
        oled.rect(4, 50, 122, 57, OLED_STROKE);
        oled.rect(4, 50, (level * 118) + 4, 57, OLED_FILL || OLED_STROKE);
        oled.update();
    }

    void DrawCalibrationInstructions()
    {
        oled.clear();
        oled.setScale(1);

        oled.setCursorXY(11, 6);
        oled.print(F("Gently Rotate the"));

        oled.setCursorXY(6, 20);
        oled.print(F("IMU While Octoslime"));

        oled.setCursorXY(30, 34);
        oled.print(F("Takes Samples"));

        oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.update();
    }


    void DrawCalibrationComplete()
    {
        oled.setScale(1);
        oled.setCursorXY(30, 33);
        oled.print(F("Calibration"));
        oled.setCursorXY(42, 46);
        oled.print(F("Complete"));

        oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.update();
    }

    void DrawCalibrationAborted()
    {
        oled.setScale(2);
        oled.update();
        oled.setCursorXY(18, 35);
        oled.print(F("Canceled"));
        // oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.update();
    }

    void MainUIFrame()
    {
        oled.clear();
        oled.update();

        for (uint8_t X = 0; X < 8; X++)
        {
            oled.rect(7 + (X * 15), 4, 17 + (X * 15), 8, OLED_STROKE);   // bank 1
            oled.rect(7 + (X * 15), 11, 17 + (X * 15), 15, OLED_STROKE); // bank 2
        }

        oled.setCursorXY(18, 25);
        oled.setScale(2);
        oled.print(F("Octo"));

        oled.setCursorXY(75, 33);
        oled.setScale(1);
        oled.print(F("Slime"));

        DrawFrame();
    }

    void SetIMUStatus(uint8 imuID, bool Status)
    {
        uint8_t UIOffset = imuID % 8;

        if (imuID < 8)
        {
            oled.rect(7 + (UIOffset * 15), 4, 17 + (UIOffset * 15), 8, (Status) ? OLED_FILL : OLED_STROKE); // bank 1
        }
        else
        {
            oled.rect(7 + (UIOffset * 15), 11, 17 + (UIOffset * 15), 15, (Status) ? OLED_FILL : OLED_STROKE); // bank 2
        }
        oled.update();
    }

    void DrawFrame(void)
    {
        oled.roundRect(0, 0, 127, 63, OLED_STROKE);
        oled.line(1, 45, 126, 45, OLED_STROKE);
        oled.line(1, 20, 126, 20, OLED_STROKE);
        oled.update();
    }

    void SetImuCount(uint8 IMUs)
    {

        oled.setScale(1);
        oled.roundRect(0, 51, 127, 63, OLED_CLEAR);
        oled.printf("%u  Sensors Active", IMUs);
        oled.setCursorXY(15, 51);
        DrawFrame();
        oled.update();
    }

    void SetMessage(uint8 MessageID)
    {

        oled.setScale(1);
        oled.roundRect(0, 51, 127, 63, OLED_CLEAR);

        switch (MessageID)
        {
        case 1:
            oled.setCursorXY(4, 51);
            oled.print(F("Scanning For Sensors"));
            break;

        case 2:
            oled.setCursorXY(10, 51);
            oled.print(F("Connecting To WIFI"));
            break;

        case 3:
            oled.setCursorXY(15, 51);
            oled.print(F("WIFI Connect FAILED"));
            break;

        case 4:
            oled.setCursorXY(5, 51);
            oled.print(F("Connecting To Server"));
            break;

        case 5:
            oled.setCursorXY(38, 51);
            oled.print(F("Server lost"));
            break;

        case 6:
            oled.setCursorXY(4, 51);
            oled.print(F("Scanning For Haptics"));
            break;
        }

        DrawFrame();
        oled.update();
    }

}