// Controls
//     throttle      pitch
// yaw     ┼          ┼    roll

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

#define recvPin1 2
#define recvPin2 4
#define recvPin3 7
#define recvPin4 8

#define signalPin1 3
#define signalPin2 5
#define signalPin3 6
#define signalPin4 9

#define RAD_TO_DEGREE 57.29
#define READINGS 11

#define GYRO_PART 0.990
#define ACC_PART 0.010

Servo motor1, motor2, motor3, motor4;

void callibrate();
void initalize_imu();
bool imu_read();
void compute_gyro_angles();
void compute_accelerometer_values();
void compute_complimentary_filter_angles_values();
float mapValue(float v,float range1,float range2,float min1,float max1);
void rc_read_values();
void fc_process();

struct Vector2 {
  float x;
  float y;
};

struct Vector3 {
  float x;
  float y;
  float z; 
};

Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

Vector3 angles, acc_angle, gyro_angles, rates, acc_filtered;

float ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET, GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET;


//Gyro rates and angle
uint32_t gyro_last_update = micros();
float delta_t;

//Mean Filter
float input;
float history[READINGS][3]={0};
int pointer= 0;

float throttle, roll, pitch, yaw;
float roll_adjust, pitch_adjust, yaw_adjust;

void setup(void) {
  Serial.begin(115200);

  pinMode(recvPin1,INPUT);
  pinMode(recvPin2,INPUT);
  pinMode(recvPin3,INPUT);
  pinMode(recvPin4,INPUT);

  motor1.attach(signalPin1);
  motor2.attach(signalPin2);
  motor3.attach(signalPin3);
  motor4.attach(signalPin4);
  
  initalize_imu();

  callibrate();

  delay(100);
 
}

void loop() {
  
  while(!imu_read());
  rc_read_values();
  fc_process();
  delay(10);
}

//SETUP
void initalize_imu(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  
}

void callibrate(){
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;

  for(int i=0;i<10;i++){
    mpu.getEvent(&a, &g, &temp);
  }
  
  int readings=30;
  for(int i=0;i<readings;i++){

    mpu.getEvent(&a, &g, &temp);
    acc_x+=a.acceleration.x;
    acc_y+=a.acceleration.y;
    acc_z+=a.acceleration.z;

    gyro_x+=g.gyro.x;
    gyro_y+=g.gyro.y;
    gyro_z+=g.gyro.z;
  }
  
  ACC_X_OFFSET = -acc_x/readings;
  ACC_Y_OFFSET = -acc_y/readings;
  ACC_Z_OFFSET = 1-(acc_z/readings);

  GYRO_X_OFFSET = -gyro_x/readings;
  GYRO_Y_OFFSET = -gyro_y/readings;
  GYRO_Z_OFFSET = -gyro_z/readings;
}

//LOOP
bool imu_read(){
  //GET VALUES
  mpu.getEvent(&a, &g, &temp);

  //PROCESS VALUES
  compute_gyro_angles();
  compute_accelerometer_values();
  compute_complimentary_filter_angles_values();

  return true;
}

void compute_gyro_angles(){
  // DEBUG
  // mpu6050_read_gyro(&gyro_rates);
  
  rates.x = g.gyro.x + GYRO_X_OFFSET;
  rates.y = g.gyro.y + GYRO_Y_OFFSET;
  rates.z = g.gyro.z + GYRO_Z_OFFSET;
  
  delta_t = (micros() - gyro_last_update) / 1000;
  delta_t = delta_t / 1000;
  
  gyro_angles.x +=(rates.x * delta_t * RAD_TO_DEGREE); // }
  gyro_angles.y +=(rates.y * delta_t * RAD_TO_DEGREE); // }- not used anywhere kept for analysis
  gyro_angles.z +=(rates.z * delta_t * RAD_TO_DEGREE); // }

  // DEBUG
  // Serial.print("gyro_angle:");
  // Serial.print(gyro_angles.x);
  // Serial.print("\ty-angle:");
  // Serial.print(gyro_angles.y);
  // Serial.print("\tz-angle:");
  // Serial.println(gyro_angles.z);

  gyro_last_update = micros();

}

void compute_accelerometer_values(){
  filter_accelerometer_values(&acc_filtered);

  float x, y, z;
  x = acc_filtered.x;
  y = acc_filtered.y;
  z = acc_filtered.z;

  acc_angle.x = atan2(y, z) * RAD_TO_DEGREE;
  acc_angle.y = atan2(-x, sqrt(y*y +z*z)) * RAD_TO_DEGREE;
  
  // DEBUG
  // Serial.print("\tacc_angle: ");
  // Serial.print(acc_angle.x);
  // Serial.print("\tY: ");
  // Serial.println(acc_angle.y);
}

void filter_accelerometer_values(Vector3 *acc_filtered){
   history[pointer][0]=a.acceleration.x;
   history[pointer][1]=a.acceleration.y;
   history[pointer][2]=a.acceleration.z;
   if(pointer==READINGS-1){
     pointer=0;
   }
   else{
     pointer++;
   }
        
   float sum_x=0, sum_y=0, sum_z=0;
   for(int i=0;i<READINGS;i++){
     sum_x+=history[i][0];
     sum_y+=history[i][1];
     sum_z+=history[i][2];
   }
   acc_filtered->x=sum_x/READINGS;
   acc_filtered->y=sum_y/READINGS;
   acc_filtered->z=sum_z/READINGS;

}

void compute_complimentary_filter_angles_values(){
  angles.x = GYRO_PART * (angles.x + (rates.x * delta_t)) + ACC_PART  * acc_angle.x;
  angles.y = GYRO_PART * (angles.y + (rates.y * delta_t)) + ACC_PART  * acc_angle.y;
  angles.z = GYRO_PART * (angles.z + (rates.z * delta_t)) + ACC_PART  * acc_angle.z;  // TEST THIS

  // DEBUG
  // Serial.print("\tcomplimentary: ");
  // Serial.println(angles.x);

}

float mapValue(float v,float range1,float range2,float min1,float max1){
    v=map(v, range1, range2, min1, max1);
    if(v>max1) v=max1;
    else if(v<min1) v=min1;

    return v;
}

void rc_read_values(){
  roll      = pulseIn(recvPin1,HIGH);
  pitch     = pulseIn(recvPin2,HIGH);
  throttle  = pulseIn(recvPin3,HIGH);
  yaw       = pulseIn(recvPin4,HIGH);
  
  roll    = mapValue(roll,1110.0, 1840.0,-25.0,25.0);
  pitch   = mapValue(pitch,1205.0, 1840.0,-25.0,25.0);
  throttle= mapValue(throttle,1180.0, 1800.0,1000.0,2000.0);
  yaw     = mapValue(yaw,1880.0, 1115.0,-50.0,50.0);

}

#define KP 2.0
#define KI 2.0
void fc_process(){
  float error_roll = roll - angles.x; // desired_roll(i.e. reading) - current_roll(i.e. angle.x)  
  float error_pitch = pitch - angles.y; // angle.y
  float error_yaw = yaw - angles.z;  // angle.z

  float proptional, integral;
    
  proptional = KP * error_roll;
  integral += KI * error_roll * delta_t;
  roll_adjust = proptional + integral;

  proptional = KP * error_pitch;
  integral += KI * error_pitch * delta_t;
  pitch_adjust = proptional + integral;
  
  proptional = KP * error_pitch;
  integral += KI * error_pitch * delta_t;
  yaw_adjust = proptional + integral;

  float m1 = throttle - roll_adjust - pitch_adjust - yaw_adjust;
  float m2 = throttle + roll_adjust + pitch_adjust - yaw_adjust;
  float m3 = throttle - roll_adjust + pitch_adjust + yaw_adjust;
  float m4 = throttle + roll_adjust - pitch_adjust + yaw_adjust;
  
  Serial.print("m1\t");
  Serial.print(m1);
  Serial.print("\t\tm2\t");
  Serial.print(m2);
  Serial.print("\t\tm3\t");
  Serial.print(m3);
  Serial.print("\t\tm4\t");
  Serial.println(m4);

  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);  

}
 
