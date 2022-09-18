# OctoSlime_haptic Tracker and Haptic feedback firmware for ESP8266

Firmware for ESP8266 microcontrollers and different IMU sensors to use them as a vive-like trackers in VR and support Haptic feedback.

Requires [SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server) to work with SteamVR and resolve pose.
Requires [OctoSlime Haptic Server] found in the server section of this repo to work with Haptics through OSC in VRC.


## Compatibility

The following IMUs are supported by the firmware, the firmware atuo detect and loads the correct objects.
A Mixture of up to 2 these devices can be made on any of the Octoslime nodes but they must have different addresses.
Interupt support wil be available for the BNO IMU

* BNO080 (IMU_BNO080)
  * Using any fusion in internal DMP. Doesn't have BNO085's ARVR stabilization, but still gives good results.
* ICM20948 (ICM_20948)
  * Using internal DMP to fuse Gyroscope and Accelerometer
* MPU-6050 (IMU_MPU6050)
  * Using internal DMP to fuse Gyroscope and Accelerometer

## Contributions

By contributing to this project you are placing all your code under MIT or less restricting licenses, and you certify that the code you have used is compatible with those licenses or is authored by you. If you're doing so on your work time, you certify that your employer is okay with this.
