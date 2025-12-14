#include <debug.h>

#include "sensorIMU.h"

#define ALPHA 0.98

void sensorIMU::calculate(sensorValuesStruct* values, unsigned long current, unsigned long delta) {
  if (values == nullptr)
    return;

  // _filter.getEulerAngles();

  // values->imu.pitch = _filter.getPitch();
  // values->imu.roll = _filter.getRoll();
  // // Yaw will drift without a magnetometer (MPU6050 is 6DOF only)
  // values->imu.yaw = _filter.getYaw();

  // // Accelerometer angles
  // float pitch_acc = atan2f(-values->acceleration.x, sqrtf(values->acceleration.y * values->acceleration.y + values->acceleration.z * values->acceleration.z)) * 180.0f / M_PI;
  // float roll_acc  = atan2f(values->acceleration.y, values->acceleration.z) * 180.0f / M_PI;

  // // Gyro integration
  // float pitch_gyro = pitch + values->gyroscope.x * delta;
  // float roll_gyro  = roll  + values->gyroscope.y * delta;

  // // Complementary filter
  // pitch = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_acc;
  // roll  = ALPHA * roll_gyro  + (1.0f - ALPHA) * roll_acc;
  
  // values->imu.pitch1 = pitch;
  // values->imu.roll1 = roll;

  // Serial.print(F("sensorIMU::values->imu.pitch="));
  // Serial.print(values->imu.pitch, 4);
  // Serial.print(F(" roll="));
  // Serial.print(values->imu.roll, 4);
  // Serial.print(F(" yaw="));
  // Serial.print(values->imu.yaw, 4);
  // Serial.print(F(", pitch1="));
  // Serial.print(values->imu.pitch1, 4);
  // Serial.print(F(" roll1="));
  // Serial.println(values->imu.roll1, 4);

  // // Check if the sensor is aligned with Z-up (pitch or roll must be close to 0 or 180 degrees)
  // bool isPitchLessThanTolerance = (fabs(values->imu.pitch) <= _verticalTolerance) || (fabs(fabs(values->imu.pitch) - 180.0f) <= _verticalTolerance);
  // bool isRollLessThanTolerance = (fabs(values->imu.roll) <= _verticalTolerance) || (fabs(fabs(values->imu.roll) - 180.0f) <= _verticalTolerance);
  // // Serial.print(F("sensorIMU::isPitchLessThanTolerance="));
  // // Serial.print(isPitchLessThanTolerance);
  // // Serial.print(F(" isRollLessThanTolerance="));
  // // Serial.println(isRollLessThanTolerance);
  
  // values->imu.vertical = isPitchLessThanTolerance && isRollLessThanTolerance;
  // // Serial.print(F("sensorIMU::values->imu.vertical="));
  // // Serial.println(values->imu.vertical);

  // // values->acceleration = acceleration;
  // // values->gyroscope = gyroscope;
  // // values->magnetometer = magnetometer;

  // Serial.print(values->acceleration.x);
  // Serial.print("\t");
  // Serial.print(values->acceleration.y);
  // Serial.print("\t");
  // Serial.print(values->acceleration.z);
  // Serial.print("\t");
  // Serial.print(values->gyroscope.x);
  // Serial.print("\t");
  // Serial.print(values->gyroscope.y);
  // Serial.print("\t");
  // Serial.print(values->gyroscope.z);
  // Serial.print("\t");
  // Serial.print(values->magnetometer.x);
  // Serial.print("\t");
  // Serial.print(values->magnetometer.y);
  // Serial.print("\t");
  // Serial.print(values->magnetometer.z);
  // Serial.println();
}

void sensorIMU::calibrationResetCommand() {
}

bool sensorIMU::hasMagnetometer() {
  return false;
}

sensorValuesStruct sensorIMU::initialize() {
  sensorValuesStruct values;
  readI(&values);
  pitch = 0.0;
  roll = 0.0;
  return values;
}

void sensorIMU::read(sensorValuesStruct* values, unsigned long current, unsigned long delta) {
  if (values == nullptr)
    return;

  readI(values);

  // Serial.print(F("sensorIMU::hasMagnetometer="));
  // Serial.println(hasMagnetometer());
  if (hasMagnetometer())
    _filter.update(values->gyroscope.x, values->gyroscope.y, values->gyroscope.z, 
      values->acceleration.x, values->acceleration.y, values->acceleration.z, 
      values->magnetometer.x, values->magnetometer.y, values->magnetometer.z, 
      current, delta);
  else {
    // Serial.print(F("sensorIMU::gyroscope.x="));
    // Serial.print(gyroscope.x, 4);
    // Serial.print(F(" y="));
    // Serial.print(gyroscope.y, 4);
    // Serial.print(F(" z="));
    // Serial.println(gyroscope.z, 4);
    _filter.updateIMU(values->gyroscope.x, values->gyroscope.y, values->gyroscope.z, 
      values->acceleration.x, values->acceleration.y, values->acceleration.z, 
      current, delta);
  }

  calculate(values, current, delta);
}

void sensorIMU::readI(sensorValuesStruct* values) {
  Serial.println(F("sensorIMU::readI"));
  if (values == nullptr) {
    Serial.println(F("sensorIMU::readI.wtf?!"));
    return;
  }

  // accelerometerValues acceleration1 = readAccelerometer();
  // gyroscopeValues gyroscope1 = readGyroscope(false);
  // magnetometerValues magnetometer1 = readMagnetometer(false);

  // accelerometerValues acceleration;
  // gyroscopeValues gyroscope;
  // magnetometerValues magnetometer;
  
  // memcpy(&acceleration, &acceleration1, sizeof(accelerometerValues));
  // memcpy(&gyroscope, &gyroscope1, sizeof(gyroscopeValues));
  // memcpy(&magnetometer, &magnetometer1, sizeof(magnetometerValues));

  // readAccelerometer(&values->acceleration);
  // readGyroscope(&values->gyroscope, false);
  // readMagnetometer(&values->magnetometer, false);

  Serial.println(F("sensorIMU::readI.default.acceleration"));
  values->acceleration.x = 0.0;
  values->acceleration.y = 0.0;
  values->acceleration.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("acceleration.x"), values->acceleration.x);
  debug(F("acceleration.y"), values->acceleration.y);
  debug(F("acceleration.z"), values->acceleration.z);
#endif

  Serial.println(F("sensorIMU::readI.default.gyroscope"));
  values->gyroscope.x = 0.0;
  values->gyroscope.y = 0.0;
  values->gyroscope.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("gyroscope.x"), values->gyroscope.x);
  debug(F("gyroscope.y"), values->gyroscope.y);
  debug(F("gyroscope.z"), values->gyroscope.z);
#endif

  Serial.println(F("sensorIMU::readI.default.magnetometer"));
  values->magnetometer.x = 0.0;
  values->magnetometer.y = 0.0;
  values->magnetometer.z = 0.0;

#if defined(DEBUG_SENSOR)
  debug(F("magnetometer.x"), values->magnetometer.x);
  debug(F("magnetometer.y"), values->magnetometer.y);
  debug(F("magnetometer.z"), values->magnetometer.z);
#endif

  // Serial.print(F("sensorIMU::acceleration.x="));
  // Serial.print(acceleration.x, 4);
  // Serial.print(F(" y="));
  // Serial.print(acceleration.y, 4);
  // Serial.print(F(" z="));
  // Serial.println(acceleration.z, 4);

  // Serial.print(F("sensorIMU::gyroscope.x="));
  // Serial.print(gyroscope.x, 4);
  // Serial.print(F(" y="));
  // Serial.print(gyroscope.y, 4);
  // Serial.print(F(" z="));
  // Serial.println(gyroscope.z, 4);
}

// void sensorIMU::readAccelerometer(accelerometerValues* values, bool update) {
//   if (values == nullptr)
//     return;

//   readAccelerometerI(values, update);

// // #if defined(KALMAN) && defined(KALMAN_ACCELEROMETER)
// //         float value = _kalmanAccelerometerX.kalmanCalc(values.x);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanAccelerometerX="));
// //         Serial.println(value);
// //   #endif
// //         values.x = value;

// //         value = _kalmanAccelerometerY.kalmanCalc(values.y);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanAccelerometerY="));
// //         Serial.println(value);
// //   #endif
// //         values.y = value;
        
// //         value = _kalmanAccelerometerZ.kalmanCalc(values.z);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanAccelerometerZ="));
// //         Serial.println(value);
// //   #endif
// //         values.z = value;
// // #endif
// }

// void sensorIMU::readAccelerometerI(accelerometerValues* values, bool update) {
//   if (values == nullptr)
//     return;

//   values->x = 0.0;
//   values->y = 0.0;
//   values->z = 0.0;

// #if defined(DEBUG_SENSOR)
//   debug(F("accelerometer.x"), values->x);
//   debug(F("accelerometer.y"), values->y);
//   debug(F("accelerometer.z"), values->z);
// #endif
// }

// void sensorIMU::readGyroscope(gyroscopeValues* values, bool update) {
//   if (values == nullptr)
//     return;

//   readGyroscopeI(values, update);

// // #if defined(KALMAN) && defined(KALMAN_GYRO)
// //         float value = _kalmanGyroX.kalmanCalc(values.x);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanGyroX="));
// //         Serial.println(value);
// //   #endif
// //         values.x = value;

// //         value = _kalmanGyroY.kalmanCalc(values.y);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanGyroY="));
// //         Serial.println(value);
// //   #endif
// //         values.y = value;
        
// //         value = _kalmanGyroZ.kalmanCalc(values.z);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanGyroZ="));
// //         Serial.println(value);
// //   #endif
// //         values.z = value;
// // #endif
// }

// void sensorIMU::readGyroscopeI(gyroscopeValues* values, bool update) {
//   if (values == nullptr)
//     return;

//   gyroscopeValues values;
//   values->x = 0.0;
//   values->y = 0.0;
//   values->z = 0.0;

// #if defined(DEBUG_SENSOR)
//   debug(F("gyroscopeV.x"), values->x);
//   debug(F("gyroscopeV.y"), values->y);
//   debug(F("gyroscopeV.z"), values->z);
// #endif
// }

// void sensorIMU::readMagnetometer(magnetometerValues* values, bool update) {
//   if (values == nullptr)
//     return;

//   readMagnetometerI(values, update);

// // #if defined(KALMAN) && defined(KALMAN_MEGNETOMETER)
// //         float value = _kalmanMagnetometerX.kalmanCalc(values->x);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanMagnetometerX="));
// //         Serial.println(value);
// //   #endif
// //         values->x = value;

// //         value = _kalmanMagnetometerY.kalmanCalc(values->y);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanMagnetometerY="));
// //         Serial.println(value);
// //   #endif
// //         values->y = value;
        
// //         value = _kalmanMagnetometerZ.kalmanCalc(values->z);
// //   #if defined(DEBUG_SENSOR)
// //         Serial.print(F("_kalmanMagnetometerZ="));
// //         Serial.println(value);
// //   #endif
// //         values->z = value;
// // #endif
// }

// void sensorIMU::readMagnetometerI(magnetometerValues* values, bool update) {
//   if (values == nullptr)
//     return;

//   values->x = 0.0;
//   values->y = 0.0;
//   values->z = 0.0;

// #if defined(DEBUG_SENSOR)
//   debug(F("magnetometer.x"), values->x);
//   debug(F("magnetometer.y"), values->y);
//   debug(F("magnetometer.z"), values->z);
// #endif
// }

int8_t sensorIMU::setup(TwoWire* wire, uint8_t calibrationId, uint8_t calibrationStatusId, int gyro_config_num, int acc_config_num) {
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