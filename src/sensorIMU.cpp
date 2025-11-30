#include <debug.h>

#include "sensorIMU.h"

void sensorIMU::calibrationResetCommand() {
}

bool sensorIMU::hasMagnetometer() {
  return false;
}

sensorValuesStruct sensorIMU::initialize() {
  sensorValuesStruct values;
  values.acceleration = readAccelerometer(true);
  values.gyroscope = readGyroscope(false);
  values.magnetometer = readMagnetometer(false);
  return values;
}

sensorValuesStruct sensorIMU::read(unsigned long current, unsigned long delta) {
  sensorValuesStruct values;
  values.acceleration = readAccelerometer();
  values.gyroscope = readGyroscope(false);
  values.magnetometer = readMagnetometer(false);

  if (hasMagnetometer())
    _filter.update(values.gyroscope.x, values.gyroscope.y, values.gyroscope.z, 
      values.acceleration.x, values.acceleration.y, values.acceleration.z, 
      values.magnetometer.x, values.magnetometer.y, values.magnetometer.z);
  else
    _filter.updateIMU(values.gyroscope.x, values.gyroscope.z, values.gyroscope.z, values.acceleration.x, values.acceleration.y, values.acceleration.z);

  values.imu.pitch = _filter.getPitch();
  values.imu.roll = _filter.getRoll();
  // Yaw will drift without a magnetometer (MPU6050 is 6DOF only)
  values.imu.yaw = _filter.getYaw();

  // Check if the sensor is aligned with Z-up (pitch or roll must be close to 0 or 180 degrees)
  bool isPitchLessThanTolerance = (fabs(values.imu.pitch) <= _verticalTolerance) || (fabs(fabs(values.imu.pitch) - 180.0f) <= _verticalTolerance);
  bool isRollLessThanTolerance = (fabs(values.imu.roll) <= _verticalTolerance) || (fabs(fabs(values.imu.roll) - 180.0f) <= _verticalTolerance);
  
  values.imu.vertical = isPitchLessThanTolerance && isRollLessThanTolerance;

  return values;
}

accelerometerValues sensorIMU::readAccelerometer(bool update) {
  accelerometerValues values = readAccelerometerI(update);

// #if defined(KALMAN) && defined(KALMAN_ACCELEROMETER)
//         float value = _kalmanAccelerometerX.kalmanCalc(values.x);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanAccelerometerX="));
//         Serial.println(value);
//   #endif
//         values.x = value;

//         value = _kalmanAccelerometerY.kalmanCalc(values.y);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanAccelerometerY="));
//         Serial.println(value);
//   #endif
//         values.y = value;
        
//         value = _kalmanAccelerometerZ.kalmanCalc(values.z);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanAccelerometerZ="));
//         Serial.println(value);
//   #endif
//         values.z = value;
// #endif

  return values;
}

accelerometerValues sensorIMU::readAccelerometerI(bool update) {
  accelerometerValues values;
  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("accelerometer.x"), values.x);
  debug(F("accelerometer.y"), values.y);
  debug(F("accelerometer.z"), values.z);
#endif

  return values;
}

gyroscopeValues sensorIMU::readGyroscope(bool update) {
  gyroscopeValues values = readGyroscopeI(update);

// #if defined(KALMAN) && defined(KALMAN_GYRO)
//         float value = _kalmanGyroX.kalmanCalc(values.x);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanGyroX="));
//         Serial.println(value);
//   #endif
//         values.x = value;

//         value = _kalmanGyroY.kalmanCalc(values.y);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanGyroY="));
//         Serial.println(value);
//   #endif
//         values.y = value;
        
//         value = _kalmanGyroZ.kalmanCalc(values.z);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanGyroZ="));
//         Serial.println(value);
//   #endif
//         values.z = value;
// #endif

  return values;
}

gyroscopeValues sensorIMU::readGyroscopeI(bool update) {
  gyroscopeValues values;
  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("gyroscopeV.x"), values.x);
  debug(F("gyroscopeV.y"), values.y);
  debug(F("gyroscopeV.z"), values.z);
#endif

  return values;
}

magnetometerValues sensorIMU::readMagnetometer(bool update) {
  magnetometerValues values = readMagnetometerI(update);

// #if defined(KALMAN) && defined(KALMAN_MEGNETOMETER)
//         float value = _kalmanMagnetometerX.kalmanCalc(values.x);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanMagnetometerX="));
//         Serial.println(value);
//   #endif
//         values.x = value;

//         value = _kalmanMagnetometerY.kalmanCalc(values.y);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanMagnetometerY="));
//         Serial.println(value);
//   #endif
//         values.y = value;
        
//         value = _kalmanMagnetometerZ.kalmanCalc(values.z);
//   #if defined(DEBUG_SENSOR)
//         Serial.print(F("_kalmanMagnetometerZ="));
//         Serial.println(value);
//   #endif
//         values.z = value;
// #endif

  return values;
}

magnetometerValues sensorIMU::readMagnetometerI(bool update) {
  magnetometerValues values;
  values.x = 0.0;
  values.y = 0.0;
  values.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("magnetometer.x"), values.x);
  debug(F("magnetometer.y"), values.y);
  debug(F("magnetometer.z"), values.z);
#endif

  return values;
}

void sensorIMU::sleep() {
  Serial.println(F("\tSleep sensor IMU..."));
  
  // _qmi.disableGyroscope();
  // _qmi.disableAccelerometer();

  Serial.println(F("\t...sensor IMU sleep successful."));
}

void sensorIMU::setVerticalTolerance(float tolerance) {
  _verticalTolerance = tolerance;
}