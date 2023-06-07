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

#define min_forward_threshold 0.8
#define max_effective_ang 5

#define forward_valid_buffer_size 30
float forward_valid_buffer[forward_valid_buffer_size][9] = {0};

#define dot_valid_buffer_size 15
bool dot_valid_buffer[dot_valid_buffer_size] = {0};

#define acc_buffer_size 30
float acc_buffer[acc_buffer_size][3] = {0};

#define max_dot_difference 0.9
#define valid_norm 0.00001

int rotate_state = 0; // initial state =0, turning clock wise = 1, turning counter clock wise = -1
int positive_count = 0;
int negative_count = 0;
int positive_count_1 = 0;
int negative_count_1 = 0;
float stop_count = 0;
float rotating_time = 0;
#define jog_activate_norm 0.005
#define jog_deactivate_norm 0.0001
#define jog_activate_cross 0.65
#define jog_deactivate_cross 0.66
#define jog_activate_cross_1 0.65
#define jog_deactivate_cross_1 0.6
#define activate_time 10
#define deactivate_time 20
#define rotating_time_threshold 1000

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

bool forward_valid(float forward_queue[][9]){
  float temp_forward_queue[forward_valid_buffer_size][9];
  for (int i = 1; i < forward_valid_buffer_size; i++){
    for (int j=0; j <9; j++){
        temp_forward_queue[i][j] = forward_queue[i][j];
        temp_forward_queue[i][j] -= forward_queue[0][j];
        if (temp_forward_queue[i][j] > min_forward_threshold){return false;}
    }
  }
  return true;
}

void enqueueBool(bool Queue[], bool new_data){
  for (int i = 0; i < dot_valid_buffer_size-1; i++){
    Queue[i] = Queue[i+1];
  }
  Queue[dot_valid_buffer_size-1] = new_data;
}

void enqueue2D_size3(int queue_len, float Queue[][3], float new_data[]){
  for (int i = 0; i < queue_len-1; i++){
    for (int j=0; j < 9; j++){
        Queue[i][j] = Queue[i+1][j];
    }
  }
  for (int j=0; j < 3; j++){
    Queue[queue_len-1][j] = new_data[j];
  }
}

void enqueue2D_size9(int queue_len, float Queue[][9], float new_data[]){
  for (int i = 0; i < queue_len-1; i++){
    for (int j=0; j < 9; j++){
        Queue[i][j] = Queue[i+1][j];
    }
  }
  for (int j=0; j < 9; j++){
    Queue[queue_len-1][j] = new_data[j];
  }
}

bool angle_valid(float v1[3], float v2[3]){
  float ang = angle(v1, v2);
  if (ang > max_effective_ang){return false;}
  else{return true;}
}

float normVector(float v[3]){
  float norm = 0.0;
  for (int i = 0; i < 3; i++){
    norm = norm + pow(v[i],2);
  }
  norm = pow(norm,0.5);
  return norm;
}

void unitVector(float v1[3], float unitVector[3]){
  float norm = normVector(&v1[3]); 
  for (int i = 0; i < 3; i++){
    unitVector[i] = v1[i]/norm;
  }
}

float angle(float v1[3], float v2[3]) {
  float dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
  float lenv1 = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
  float lenv2 = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
  float angle = acos(dot/sqrt(lenv1 * lenv2));
  return angle;
}

void cross(float v1[3], float v2[3], float cross_product[3]){
  cross_product[0] = v1[1]*v2[2]-v1[2]*v2[1];
  cross_product[1] = v1[2]*v2[0]-v1[0]*v2[2];
  cross_product[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

void update_state(float inertia[3], int valid, int *positive_count, int *negative_count,int *positive_count_1, int *negative_count_1,float *stop_count, int *state,float rtx[9]){
  float normalize_iner;
  float normalize_iner_1;
  normalize_iner = inertia[0]/normVector(inertia);
  normalize_iner_1 = inertia[2]/normVector(inertia);
  if (valid){
    if(*state ==0){
      if (normVector(inertia) > jog_activate_norm and normalize_iner > jog_activate_cross and rtx[6] > 0.5){
        rotating_time = millis();
        *positive_count += 1;
//        *negative_count = 0;
//        *positive_count_1 = 0;
//        *negative_count_1 = 0;
        if (*positive_count == activate_time){
          *state = 1;
          *negative_count = 0;
          *positive_count_1 = 0;
          *negative_count_1 = 0;
          *positive_count = 0;
          *stop_count = 0;
        }
      }
      if (normVector(inertia) > jog_activate_norm and normalize_iner < -jog_activate_cross and rtx[6] > 0.5){
        rotating_time = millis();
        *negative_count += 1;
        *positive_count = 0;
        *positive_count_1 = 0;
        *negative_count_1 = 0;
        if (*negative_count == activate_time){
          *state = 2;
          *negative_count = 0;
          *positive_count_1 = 0;
          *negative_count_1 = 0;
          *positive_count = 0;
          *stop_count = 0;
        }
      }
      if (normVector(inertia) > jog_activate_norm and normalize_iner_1 > jog_activate_cross_1){
        rotating_time = millis();
        *positive_count = 0;
        *negative_count = 0;
        *positive_count_1 += 1;
        *negative_count_1 = 0;
        if (*positive_count_1 == activate_time){
          *state = 3;
          *negative_count = 0;
          *positive_count_1 = 0;
          *negative_count_1 = 0;
          *positive_count = 0;
          *stop_count = 0;
        }
      }
//      else if (normVector(inertia) > jog_activate_norm and normalize_iner_1 < -jog_activate_cross_1 and rtx[6] > -0.5 and rtx[6] < 0.5){
//        rotating_time = millis();
////        *positive_count = 0;
////        *negative_count = 0;
//        *negative_count_1 += 1;
////        *positive_count_1 = 0;
//        if (*negative_count_1 == activate_time){
//          *state = 4;
//          *negative_count = 0;
//          *positive_count_1 = 0;
//          *negative_count_1 = 0;
//          *positive_count = 0;
//          *stop_count = 0;
//        }
//      }
//    }
    else if(*state ==1){
      if ((normVector(inertia) > jog_activate_norm and normalize_iner > jog_activate_cross and rtx[6] > 0.5)==false){
        *stop_count += 1;
        if (*stop_count >= deactivate_time or millis()-rotating_time>rotating_time_threshold){
          *state = 0;
          *positive_count = 0;
          *negative_count = 0;
          *positive_count_1 = 0;
          *negative_count_1 = 0;
          *stop_count = 0 ;
          forward_valid_buffer[forward_valid_buffer_size][9] = {0};
          dot_valid_buffer[dot_valid_buffer_size] = {0};
          acc_buffer[acc_buffer_size][3] = {0};
        }
      }
      else{
        rotating_time = millis();
        if(*stop_count >0){
          *stop_count = 0;
        }
      }
    }
    else if(*state == 2){
      if ((normVector(inertia) > jog_activate_norm and normalize_iner < -jog_activate_cross and rtx[6] > 0.5 )==false){
        *stop_count += 1;
        if (*stop_count >= deactivate_time or millis()-rotating_time>rotating_time_threshold){
          *state = 0;
          *positive_count = 0;
          *negative_count = 0;
          *positive_count_1 = 0;
          *negative_count_1 = 0;
          *stop_count =0;
          forward_valid_buffer[forward_valid_buffer_size][9] = {0};
          dot_valid_buffer[dot_valid_buffer_size] = {0};
          acc_buffer[acc_buffer_size][3] = {0};
        }
      }
      else{
        rotating_time = millis();
        if(*stop_count >0){
          *stop_count = 0;
        }
      }
    }
    else if(*state ==3){
      if ((normVector(inertia) > jog_activate_norm and normalize_iner_1 > jog_activate_cross_1)==false){
        *stop_count += 1;
        if (*stop_count >= deactivate_time or millis()-rotating_time>rotating_time_threshold){
          *state = 0;
          *positive_count = 0;
          *negative_count = 0;
          *positive_count_1 = 0;
          *negative_count_1 = 0;
          *stop_count = 0 ;
          forward_valid_buffer[forward_valid_buffer_size][9] = {0};
          dot_valid_buffer[dot_valid_buffer_size] = {0};
          acc_buffer[acc_buffer_size][3] = {0};
        }
      }
      else{
        rotating_time = millis();
        if(*stop_count >0){
          *stop_count = 0;
        }
      }
    }
//    else if(*state == 4){
//      if ((normVector(inertia) > jog_activate_norm and normalize_iner_1 < -jog_activate_cross_1 and rtx[6] > -0.5 and rtx[6] < 0.5)==false){
//        *stop_count += 1;
//        if (*stop_count >= deactivate_time or millis()-rotating_time>rotating_time_threshold){
//          *state = 0;
//          *positive_count = 0;
//          *negative_count = 0;
//          *positive_count_1 = 0;
//          *negative_count_1 = 0;
//          *stop_count =0;
//          forward_valid_buffer[forward_valid_buffer_size][9] = {0};
//          dot_valid_buffer[dot_valid_buffer_size] = {0};
//          acc_buffer[acc_buffer_size][3] = {0};
//        }
//      }
//      else{
//        rotating_time = millis();
//        if(*stop_count >0){
//          *stop_count = 0;
//        }
//      }
    }
  }
  if (!valid){
    *stop_count += 0.5;
    if (*positive_count >0){positive_count = 0;}
    if (*negative_count >0){negative_count = 0;}
    if (*positive_count_1 >0){positive_count_1 = 0;}
    if (*negative_count_1 >0){negative_count_1 = 0;}
    if (*stop_count >= deactivate_time or millis() - rotating_time > rotating_time_threshold){
      *state = 0;
      *stop_count = 0;
      forward_valid_buffer[forward_valid_buffer_size][9] = {0};
      dot_valid_buffer[dot_valid_buffer_size] = {0};
      acc_buffer[acc_buffer_size][3] = {0};
    }
  }
}
float send_time = micros();
float pre_IMU_X[3] = {0};
float pre_IMU_Y[3] = {0};
float pre_IMU_Z[3] = {0};
int up_count = 0;
int up_state = 0;
int down_count = 0;
int down_state = 0;
int left_count = 0;
int right_count = 0;
  
void loop() {
  // read IMU data
  if (!IMU.readGyroscope(gyo) || !IMU.readAcceleration(acc) || !IMU.readMagneticField(mag)) {
    printf("IMU read data error!\n\n");
    return;
  }
  // swape axis
  delay(4);
  gyo = FusionAxesSwap(gyo, FusionAxesAlignmentPXNYPZ);
  acc = FusionAxesSwap(acc, FusionAxesAlignmentPXNYPZ);
  mag = FusionAxesSwap(mag, FusionAxesAlignmentNXNYPZ);

  // calibration
  gyo = FusionCalibrationInertial(gyo, GyroMisalignment, GyroSensitivity, GyroOffset);
  acc = FusionCalibrationInertial(acc, AccMisalignment, AccSensitivity, AccOffset);
  mag = FusionCalibrationMagnetic(mag, SoftIronMatrix, HardIronOffset);

  FusionAhrsUpdate(&ahrs, gyo, acc, mag, delta_time);
  
  current_time = micros();
  delta_time = (current_time - time_stamp) * 0.000001;
  time_stamp = current_time;

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
  
  quaternion2matrix(quaternion.array, rtx);
  enqueue2D_size9(forward_valid_buffer_size, forward_valid_buffer, rtx);
  enqueue2D_size3(acc_buffer_size, acc_buffer, linear_data);
  float inertia[3];
  bool inertia_valid = false;
  cross(linear_data,acceleration,inertia);
  bool dot_valid = true;
  bool new_dot_valid = false;
  
  if ( normVector(acc_buffer[acc_buffer_size-1]) >= valid_norm and normVector(acc_buffer[0]) >= valid_norm){
    float new_acc[3];
    float old_acc[3];
    float dot_value = 0;
    for (int i = 0; i <3; i++){
      new_acc[i] = acc_buffer[acc_buffer_size-1][i]/normVector(acc_buffer[acc_buffer_size-1]);
      old_acc[i] = acc_buffer[0][i]/normVector(acc_buffer[0]);
      dot_value += new_acc[i] * old_acc[i];
    }          
    if (dot_value<max_dot_difference and dot_value > -max_dot_difference){
      new_dot_valid = true;
    }
  }    
  enqueueBool(dot_valid_buffer, new_dot_valid);
  for (int i=0; i<dot_valid_buffer_size; i++){
    if (dot_valid_buffer[i] == false){
      dot_valid = false;
      break;
    }
  } 

  float IMU_X[3] = {rtx[0],rtx[3],rtx[6]};
  float IMU_Y[3] = {rtx[1],rtx[4],rtx[7]};
  float IMU_Z[3] = {rtx[2],rtx[5],rtx[8]};

  float inertia_X[3];
  float inertia_Y[3];
  float inertia_Z[3];
  
  cross(IMU_X, pre_IMU_X, inertia_X);
  cross(IMU_Y, pre_IMU_Y, inertia_Y);
  cross(IMU_Z, pre_IMU_Z, inertia_Z);
  
  pre_IMU_X[0] = IMU_X[0];
  pre_IMU_X[1] = IMU_X[1];
  pre_IMU_X[2] = IMU_X[2];
  pre_IMU_Y[0] = IMU_Y[0];
  pre_IMU_Y[1] = IMU_Y[1];
  pre_IMU_Y[2] = IMU_Y[2];
  pre_IMU_Z[0] = IMU_Z[0];
  pre_IMU_Z[1] = IMU_Z[1];
  pre_IMU_Z[2] = IMU_Z[2];
  
  up_count += 1;
  down_count += 1;
  
      
  if (angle_valid(acceleration,linear_data) and dot_valid and forward_valid(forward_valid_buffer)){   
    valid = true;
  }
//  Serial.println(forward_valid(forward_valid_buffer));
  if (IMU_X[2] < -0.5){
    up_state = 1;
    up_count = 0;}
  if (IMU_X[2] > 0.8 and up_state == 1 and up_count <50){
    Serial.println("up");
    up_state = 0;}
  if (IMU_X[2] > 0.5){
    down_state = 1;
    down_count = 0;}
  if (IMU_X[2] < -0.8 and down_state == 1 and down_count <50){
    Serial.println("down");
    down_state = 0;}
  if (inertia_X[2] > 0.045 and inertia_Y[2] > 0.045){
    left_count += 1;
    valid = false;
    if (left_count == 10){
      Serial.println("right");
    }    
  }
  else if (inertia_X[2] < -0.045 and inertia_Y[2] < -0.045 ){
    right_count += 1;
    valid = false;
    if (right_count == 10){
      Serial.println("left");
    }
  }
  else{
    right_count = 0;
    left_count = 0;}
    
  update_state(inertia,valid,&positive_count,&negative_count,&positive_count_1,&negative_count_1,&stop_count,&rotate_state,rtx);     
  for (int i = 0; i<3; i++){
    acceleration[i] = linear_data[i];
  }
//  Serial.print(IMU_X[2]);
  if (rotate_state!=0 and current_time-send_time > 20000){
    Serial.println(rotate_state);
    send_time = current_time;
  }
}
