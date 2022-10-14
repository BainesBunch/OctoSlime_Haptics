#include "sensor.h"
#include <Icm20948.h>
#include "helper_3dmath.h"
#include "Arduino-ICM20948.h"
#include "TapDetector/TapDetector.h"

class ICM20948Sensor : public Sensor {
    public:
        ICM20948Sensor() = default;
        ~ICM20948Sensor() override = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
        void save_bias(bool repeat);
        void setupICM20948(bool auxiliary = false, uint8_t addr = 0x69);

    private:
        void i2c_scan();
        bool auxiliary {false};
        unsigned long lastData = 0;
        uint8_t addr = 0x69;
        int bias_save_counter = 0;
        uint8_t ICM_address;
        bool ICM_found = false;
        bool ICM_init = false;
        bool newData = false;
        bool newTap;
        ArduinoICM20948 icm20948;
        ArduinoICM20948Settings icmSettings;
        #ifdef ESP32
            Preferences prefs;
            Timer<> timer = timer_create_default();
        #endif
        TapDetector tapDetector;
};