

#include "globals.h"
#include "ICM20948Sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "calibration.h"
#include "configuration.h"

icm_20948_DMP_data_t data;

void ICM20948Sensor::setupSensor(uint8_t sensorId)
{
    this->sensorId = sensorId;
    this->sensorType = IMU_ICM20948;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), IMU_ROTATION)};
    this->working = false;
    this->configured = false;
}

boolean ICM20948Sensor::motionSetup()
{

    Serial.print(F("Initialising ICM20948 on MUX Channel : "));
    Serial.print(sensorId % 8);
    Serial.print(F(" Bank : "));
    Serial.print(sensorId < 8 ? "A" : "B");
    Serial.print(F(" At Address : "));
    Serial.print(addr,HEX);

    imu.begin(Wire, addr);

    if (imu.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("[ERR] ICM20948: Can't communicate with MPU, response "));
        Serial.println(imu.getWhoAmI(), HEX);
        return false;
    }

    imu.swReset();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("Software Reset returned: "));
        Serial.println(imu.statusString());
        return false;
    }

    delay(250);

    imu.sleep(false);
    imu.lowPower(false);

    Serial.println(F("Enabling DMP"));

    bool success = true;

    success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);

    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);

    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to 225Hz

    // Enable the FIFO
    success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (imu.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (imu.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

    if (success)
    {
        Serial.println(F("DMP ready"));
        dmpReady = true;
        packetSize = sizeof(icm_20948_DMP_data_t);
    }
    else
    {
        Serial.println(F("Enable DMP failed!"));
        return false;
    }
    imu.cfgIntActiveLow(true);       // Active low to be compatible with the breakout board's pullup resistor
    imu.cfgIntOpenDrain(true);       // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
    imu.cfgIntLatch(true);           // Latch the interrupt until cleared
    imu.intEnableRawDataReady(true); // enable interrupts on raw data ready

    working = true;
    configured = true;
    return false;
}

void ICM20948Sensor::motionLoop()
{

    newData = false;

    if (!dmpReady)
        return;

    if (imu.readDMPdataFromFIFO(&data) == ICM_20948_Stat_FIFONoDataAvail)
        return;

    if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {

        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {

            double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q4 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            rawQuat.x = q1;
            rawQuat.y = q2;
            rawQuat.z = q3;
            rawQuat.w = q4;
            calibrationAccuracy = data.Quat9.Data.Accuracy;
            quaternion.set(-rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w);
            quaternion *= sensorOffset;
            if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
            {
                newData = true;
                lastQuatSent = quaternion;
            }
        }

        newData = true;
    }
}



void ICM20948Sensor::startCalibration(int calibrationType)
{
#ifdef IMU_MPU6050_RUNTIME_CALIBRATION
    Serial.println(F("MPU is using automatic runtime calibration. Place down the device and it should automatically calibrate after a few seconds"));

    // Lie to the server and say we've calibrated
    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        Network::sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0);
        break;
    }

#else //! IMU_MPU6050_RUNTIME_CALIBRATION
    Serial.println("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);

    imu.setDMPEnabled(false);
    imu.CalibrateGyro(6);
    imu.CalibrateAccel(6);
    imu.setDMPEnabled(true);

    Serial.println("Calibrated!");
    Serial.println("[NOTICE] Starting offset finder");
    DeviceConfig *const config = getConfigPtr();

    switch (calibrationType)
    {
    case CALIBRATION_TYPE_INTERNAL_ACCEL:
        imu.CalibrateAccel(10);
        sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
        config->calibration.A_B[0] = imu.getXAccelOffset();
        config->calibration.A_B[1] = imu.getYAccelOffset();
        config->calibration.A_B[2] = imu.getZAccelOffset();
        saveConfig();
        break;
    case CALIBRATION_TYPE_INTERNAL_GYRO:
        imu.CalibrateGyro(10);
        sendCalibrationFinished(CALIBRATION_TYPE_INTERNAL_ACCEL, 0, PACKET_RAW_CALIBRATION_DATA);
        config->calibration.G_off[0] = imu.getXGyroOffset();
        config->calibration.G_off[1] = imu.getYGyroOffset();
        config->calibration.G_off[2] = imu.getZGyroOffset();
        saveConfig();
        break;
    }

    Serial.println("[NOTICE] Process is over");
    LEDMGR::off(CALIBRATING_LED);

#endif // !IMU_MPU6050_RUNTIME_CALIBRATION
}
