/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "mpu9250sensor.h"
// #include "GlobalVars.h"
#include "EEPROM_I2C/EEPROM_I2C.h"
#include "calibration.h"
#include "globals.h"
#include "helper_3dmath.h"
#include "magneto1.4.h"
#include "network/network.h"
#include <i2cscan.h>
// #include "mahony.h"
// #include "madgwick.h"
#if not(defined(_MAHONY_H_) || defined(_MADGWICK_H_))
#include "dmpmag.h"
#endif

// See AK8693 datasheet for sensitivity scales in different mode
// We use 16-bit continuous reading mode
#define MAG_LSB_TO_MG_8G .333f
//#define MAG_UT_LSB_16_BIT 0.15f

// 131 LSB/deg/s = 250 deg/s
#define TYPICAL_GYRO_SENSITIVITY 131
// 16384 LSB/G = 2G
#define TYPICAL_ACCEL_SENSITIVITY 16384.

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
// Gyro scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_GYRO_SENSITIVITY) / 32768.) * (PI / 180.0);
// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE = ((32768. / TYPICAL_ACCEL_SENSITIVITY) / 32768.) * SENSORS_GRAVITY_EARTH;
#endif

#define MAG_CORR_RATIO 0.02

void MPU9250Sensor::setupSensor(uint8_t sensorId)
{
    this->sensorId = sensorId;
    this->sensorType = IMU_MPU9250;
    this->sensorOffset = { Quat(Vector3(0, 0, 1), IMU_ROTATION) };
    this->working = false;
    this->configured = false;
    this->eepromAddr = sensorId < 8 ? eepromBankAddressA : eepromBankAddressB;
}

boolean MPU9250Sensor::motionSetup()
{
    boolean RetVal = false;
    // initialize device
    imu.initialize(addr);
    if (!imu.testConnection()) {
        // m_Logger.fatal("Can't connect to MPU9250 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        return false;
    }

    // m_Logger.info("Connected to MPU9250 (reported device ID 0x%02x) at address 0x%02x on MUX Channel %02x", imu.getDeviceID(), addr, sensorId % 8);

    if (imu.getMagnetometerDeviceID() != 0xFF) {
        // m_Logger.fatal("Can't connect to QMC5883L (reported ID 0x%02x) at address 0x%02x", magId, 0x0D);
    } else {
        // m_Logger.info("Connected to QMC5883L (reported ID 0x%02x) at address 0x%02x", magId, 0x0D);
    }

    {
        short calCheck = EEPROM_I2C::checkForCalibration(this->eepromAddr);
        Serial.printf("\nCalibration check: %d\n", calCheck);
        if (calCheck != 0) {
            UI::DrawCalibrationAdvice(sensorId);
            // EEPROM_I2C::clearCalibration(this->eepromAddr);
        }
    }

    int16_t ax, ay, az;

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY; // For 2G sensitivity
    // g_az = -1.f; // for calibration debugging
    if (g_az < -0.75f) {
        UI::DrawCalibrationScreen(sensorId);
        for (uint8_t CountDown = 10; CountDown > 0; CountDown--) {
            UI::DrawCalibrationContdown(CountDown);
            delay(1000);
            ESP.wdtFeed();
            imu.getAcceleration(&ax, &ay, &az);
            g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY;
            if (g_az > 0.75f)
                break;
        }

        // g_az = 1.f; // for calibration debugging
        if (g_az > 0.75f) {
            RetVal = true;
            // m_Logger.debug("Starting calibration...");
            startCalibration(0);
        } else {
            UI::DrawCalibrationAborted();
            delay(1000);
        }
    }
    return RetVal;
}

void MPU9250Sensor::calibrationSetup()
{
    // Initialize the configuration
    {
        // SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        Octo_SlimeVR::Configuration::CalibrationConfig sensorCalibration;
        sensorCalibration.type = Octo_SlimeVR::Configuration::CalibrationConfigType::MPU9250;
        EEPROM_I2C::readCalibration(eepromAddr, &sensorCalibration);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case Octo_SlimeVR::Configuration::CalibrationConfigType::MPU9250:
            m_Calibration = sensorCalibration.data.mpu9250;
            break;

        case Octo_SlimeVR::Configuration::CalibrationConfigType::NONE:
            // m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            // m_Logger.info("Calibration is advised");
            break;

        default:
            // m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            // m_Logger.info("Calibration is advised");
            break;
        }
    }

    Serial.println("[INFO] Magnetometer calibration matrix:\n{");

    for (short i = 0; i < 3; i++) {
        Serial.printf(" %f, %f, %f, %f\n", m_Calibration.M_B[i], m_Calibration.M_Ainv[0][i], m_Calibration.M_Ainv[1][i], m_Calibration.M_Ainv[2][i]);
    }
    Serial.println("}");

#if not(defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    devStatus = imu.dmpInitialize();
    if (devStatus == 0) {
        // ledManager.pattern(50, 50, 5);

        // turn on the DMP, now that it's ready
        // m_Logger.debug("Enabling DMP...");
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // m_Logger.debug("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
        working = true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // m_Logger.error("DMP Initialization failed (code %d)", devStatus);
        Serial.printf("DMP init failed, %d", devStatus);
    }
    configured = true;
#else
    working = true;
    configured = true;
#endif
}

void MPU9250Sensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ, mX, mY, mZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);
        imu.getMagnetometer(&mX, &mY, &mZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, mX, mY, mZ, 255);
    }
#endif

#if not(defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    // Update quaternion
    if (!dmpReady) {
        return;
    }
    Quaternion rawQuat {};
    if (!imu.GetCurrentFIFOPacket(fifoBuffer, imu.dmpGetFIFOPacketSize())) {
        return;
    }
    if (imu.dmpGetQuaternion(&rawQuat, fifoBuffer)) {
        return; // FIFO CORRUPTED
    }
    Quat quat(-rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w);

    getMPUScaled();

    if (Mxyz[0] == 0.0f && Mxyz[1] == 0.0f && Mxyz[2] == 0.0f) {
        return;
    }

    VectorFloat grav;
    imu.dmpGetGravity(&grav, &rawQuat);

    float Grav[] = { grav.x, grav.y, grav.z };

    if (correction.length_squared() == 0.0f) {
        correction = getCorrection(Grav, Mxyz, quat);
    } else {
        Quat newCorr = getCorrection(Grav, Mxyz, quat);

        if (!__isnanf(newCorr.w)) {
            correction = correction.slerp(newCorr, MAG_CORR_RATIO);
        }
    }

    quaternion = correction * quat;
#else
    unsigned long now = micros();
    unsigned long deltat = now - last; // seconds since last update
    last = now;
    getMPUScaled();

#if defined(_MAHONY_H_)
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
#elif defined(_MADGWICK_H_)
    madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
#endif
    quaternion.set(-q[2], q[1], q[3], q[0]);

#endif
    quaternion *= sensorOffset;

#if ENABLE_INSPECTION
    {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
    }
#endif

    if (!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU9250Sensor::getMPUScaled()
{
    float temp[3];
    int i;

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = ((float)gx - m_Calibration.G_off[0]) * GSCALE;
    Gxyz[1] = ((float)gy - m_Calibration.G_off[1]) * GSCALE;
    Gxyz[2] = ((float)gz - m_Calibration.G_off[2]) * GSCALE;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

// apply offsets (bias) and scale factors from Magneto
#if useFullCalibrationMatrix == true
    for (i = 0; i < 3; i++)
        temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
    Axyz[0] = (m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2]) * ASCALE;
    Axyz[1] = (m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2]) * ASCALE;
    Axyz[2] = (m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2]) * ASCALE;
#else
    for (i = 0; i < 3; i++)
        Axyz[i] = (Axyz[i] - m - Calibration.A_B[i]);
#endif

#else
    int16_t mx, my, mz;
    // with DMP, we just need mag data
    imu.getMagnetometer(&mx, &my, &mz);
#endif

    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    Mxyz[0] = (float)my; //
    Mxyz[1] = -(float)mx; //      CHANGE THESE DEPENDING ON HOW YOUR QMC IS INSTALLED RELATIVE TO YOUR MPU6050
    Mxyz[2] = (float)mz; //
// apply offsets and scale factors from Magneto
#if useFullCalibrationMatrix == true
    for (i = 0; i < 3; i++)
        temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
    Mxyz[0] = (m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2]) * MAG_LSB_TO_MG_8G * .1f;
    Mxyz[1] = (m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2]) * MAG_LSB_TO_MG_8G * .1f;
    Mxyz[2] = (m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2]) * MAG_LSB_TO_MG_8G * .1f;
#else
    for (i = 0; i < 3; i++)
        Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
#endif

    uint32_t t = micros();
    Mxyz[0] = f_mag_y.filter(Mxyz[0], t);
    Mxyz[1] = f_mag_x.filter(Mxyz[1], t);
    Mxyz[2] = f_mag_z.filter(Mxyz[2], t);
}

void MPU9250Sensor::startCalibration(int calibrationType)
{
    // ledManager.on();
#if not(defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    // with DMP, we just need mag data

    // constexpr uint8_t calibrationSamples = 50; // KEEP THIS AT 100 AS IT IS KNOWN TO CAUSE OOM ERRORS
    // constexpr uint8_t calibrationBatches = 10; // 5 to get 500 samples

    UI::DrawCalibrationInstructions();
    Serial.println();

    MagnetoCalibration* magneto = new MagnetoCalibration();
    for (int i = 0; i < 500; i++) {
        UI::DrawCalibrationProgress(500, i);
        int16_t mx, my, mz;
        imu.getMagnetometer(&mx, &my, &mz);
        magneto->sample(mx, my, mz);
        float rawMagFloat[3] = { (float)mx, (float)my, (float)mz };
        Network::sendRawCalibrationData(rawMagFloat, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        Serial.print(".");
        delay(50);
    }

    Serial.println("\t Sampling done");

    float M_BAinv[4][3];
    magneto->current_calibration(M_BAinv);
    delete magneto;

    // Blink calibrating led before user should rotate the sensor
    // m_Logger.info("Gently rotate the device while it's gathering magnetometer data");
    // ledManager.pattern(15, 300, 3000 / 310);

    // float* calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    // float M_BAinv[4][3];
    // int16_t mx, my, mz;
    // uint8_t i = 0;
    // for (uint8_t batch = 0; batch < calibrationBatches; batch++) {

    //     Serial.printf("new batch #%d\n", batch);

    //     for (i = 0; i < calibrationSamples; i++) {
    //         UI::DrawCalibrationProgress(calibrationSamples * calibrationBatches, i + (batch * calibrationSamples));
    //         // ledManager.on();
    //         imu.getMagnetometer(&mx, &my, &mz);
    //         Serial.printf("%d ", i);
    //         calibrationDataMag[i * 3 + 0] = my; //
    //         calibrationDataMag[i * 3 + 1] = -mx; //      CHANGE THESE DEPENDING ON HOW YOUR QMC IS INSTALLED RELATIVE TO YOUR MPU6050
    //         calibrationDataMag[i * 3 + 2] = mz; //
    //         Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
    //         // ledManager.off();
    //         ESP.wdtFeed();
    //         delay(5);
    //     }
    //     Serial.printf("batch %d done\n", batch);

    //     // m_Logger.debug("Calculating calibration data, batch %d...", batch + 1);

    //     CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    //     // free(calibrationDataMag);
    //     memset(M_BAinv, 0, 4 * 3);

    //     Serial.println("copying values");

    //     // m_Logger.debug("[INFO] Magnetometer calibration matrix batch %d:", batch + 1);
    //     // m_Logger.debug("{");
    //     if (batch == 0) {
    //         for (i = 0; i < 3; i++) {
    //             m_Calibration.M_B[i] = M_BAinv[0][i];
    //             m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
    //             m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
    //             m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
    //             // m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    //         }
    //     } else {
    //         for (i = 0; i < 3; i++) {
    //             m_Calibration.M_B[i] = (m_Calibration.M_B[i] + M_BAinv[0][i]) / (!isnan(m_Calibration.M_B[i]) ? 2 : 1);
    //             m_Calibration.M_Ainv[0][i] = (m_Calibration.M_Ainv[0][i] + M_BAinv[1][i]) / (!isnan(m_Calibration.M_Ainv[0][i]) ? 2 : 1);
    //             m_Calibration.M_Ainv[1][i] = (m_Calibration.M_Ainv[1][i] + M_BAinv[2][i]) / (!isnan(m_Calibration.M_Ainv[1][i]) ? 2 : 1);
    //             m_Calibration.M_Ainv[2][i] = (m_Calibration.M_Ainv[2][i] + M_BAinv[3][i]) / (!isnan(m_Calibration.M_Ainv[2][i]) ? 2 : 1);
    //             // m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    //         }
    //     }
    //     // m_Logger.debug("}");
    // }
    UI::DrawCalibrationScreen(sensorId);
    UI::DrawCalibrationComplete();

#else

    // m_Logger.debug("Gathering raw data for device calibration...");
    constexpr int calibrationSamples = 300;
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    // m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++) {
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
    // m_Logger.trace("Gyro calibration results: %f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    m_Calibration.G_off[0] = Gxyz[0];
    m_Calibration.G_off[1] = Gxyz[1];
    m_Calibration.G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    // m_Logger.info("Gently rotate the device while it's gathering accelerometer and magnetometer data");
    ledManager.pattern(15, 300, 3000 / 310);
    float* calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    float* calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        calibrationDataMag[i * 3 + 0] = my;
        calibrationDataMag[i * 3 + 1] = mx;
        calibrationDataMag[i * 3 + 2] = -mz;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    // m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);
    // m_Logger.debug("Finished Calculate Calibration data");
    // m_Logger.debug("Accelerometer calibration matrix:");
    // m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        // m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    // m_Logger.debug("}");
    // m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    // m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        // m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }

    // m_Logger.debug("}");

#endif

    // m_Logger.debug("Saving the calibration data");

    Serial.println("[INFO] Magnetometer calibration matrix:\n{");

    for (short i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        Serial.printf(" %f, %f, %f, %f\n", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    Serial.println("}");
    {
        // SlimeVR::Configuration::CalibrationConfig config = configuration.getCalibration(sensorId);
        // config.type = SlimeVR::Configuration::CalibrationConfigType::NONE;
        // configuration.setCalibration(sensorId, config);
        EEPROM_I2C::clearCalibration(eepromAddr);
    }

    Octo_SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = Octo_SlimeVR::Configuration::CalibrationConfigType::MPU9250;
    calibration.data.mpu9250 = m_Calibration;
    // configuration.setCalibration(sensorId, calibration);
    // configuration.save();

    // Experimental EEPROM
    EEPROM_I2C::writeCalibration(eepromAddr, calibration);

    // ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    // m_Logger.debug("Saved the calibration data");

    // m_Logger.info("Calibration data gathered");
}