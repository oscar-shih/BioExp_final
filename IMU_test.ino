//#include <LibPrintf.h>
#include "common.h"
#include "Fusion.h"
#include "LSM9DS1.h"

float acc_x, acc_y, acc_z;
float gyo_x, gyo_y, gyo_z;
float mag_x, mag_y, mag_z;

FusionVector acc;
FusionVector gyo;
FusionVector mag;

unsigned long frameNumber;
unsigned long time_stamp;
unsigned long current_time;
float delta_time;

FusionAhrs ahrs;

void setup() {
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  IMU.setContinuousMode();

  time_stamp = micros();
  frameNumber = 0;

  // init AHRS
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
    .gain = 0.5f,
    .accelerationRejection = 10.0f,
    .magneticRejection = 20.0f,
    .rejectionTimeout = 5 * 119, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);
}


void quaternion2matrix(const float Q[4], float rtx[9]){
  float q0 = Q[0];
  float q1 = Q[1];
  float q2 = Q[2];
  float q3 = Q[3];
    
  //First row of the rotation matrix
  rtx[0] = 2 * (q0 * q0 + q1 * q1) - 1;
  rtx[1] = 2 * (q1 * q2 - q0 * q3);
  rtx[2] = 2 * (q1 * q3 + q0 * q2);
    
  //Second row of the rotation matrix
  rtx[3] = 2 * (q1 * q2 + q0 * q3);
  rtx[4] = 2 * (q0 * q0 + q2 * q2) - 1;
  rtx[5] = 2 * (q2 * q3 - q0 * q1);
    
  //Third row of the rotation matrix
  rtx[6] = 2 * (q1 * q3 - q0 * q2);
  rtx[7] = 2 * (q2 * q3 + q0 * q1);
  rtx[8] = 2 * (q0 * q0 + q3 * q3) - 1;
}

void loop() {
  // read IMU data
  if (!IMU.readGyroscope(gyo) || !IMU.readAcceleration(acc) || !IMU.readMagneticField(mag)) {
    printf("IMU read data error!\n\n");
    return;
  }

  current_time = micros();
  delta_time = (current_time - time_stamp) * 0.000001;
  time_stamp = current_time;
  
  // swape axis
  gyo = FusionAxesSwap(gyo, FusionAxesAlignmentPXNYPZ);
  acc = FusionAxesSwap(acc, FusionAxesAlignmentPXNYPZ);
  mag = FusionAxesSwap(mag, FusionAxesAlignmentNXNYPZ);

  // calibration
  // gyo = FusionCalibrationInertial(gyo, GyroMisalignment, GyroSensitivity, GyroOffset);
  // acc = FusionCalibrationInertial(acc, AccMisalignment, AccSensitivity, AccOffset);
  
  mag = FusionCalibrationMagnetic(mag, SoftIronMatrix, HardIronOffset);

  FusionAhrsUpdate(&ahrs, gyo, acc, mag, delta_time);

  bool valid = false;
  const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
  const FusionEuler euler = FusionQuaternionToEuler(quaternion);
  const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
  const FusionVector linear = FusionAhrsGetLinearAcceleration(&ahrs);
  const IMU_QuaternionEularEarthLinear data = { .quaternion = quaternion, .euler = euler, .earth = earth, .linear = linear };
  float rtx[9];
  float acceleration[3];
  float linear_data[3] = {linear.array[0],linear.array[1],linear.array[2]};
  float earth_data[3] = {earth.array[0],earth.array[1],earth.array[2]};
  
  Serial.print(earth.array[0]);
  Serial.print(" ");
  Serial.print(earth.array[1]);
  Serial.print(" ");
  Serial.println(earth.array[2]);
 
}
