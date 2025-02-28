#include <Arduino.h>     // every sketch needs this
#include <Wire.h>        // instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.2.0
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>

#define HMC5883L_ADDRESS 0x1E
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float BOT_SIZE = 50;
const float tick_to_cm = (2.0 * PI * 10.0) / 1750.0;  // cm per tick
const float cm_to_tick = 250.0/(2.0* PI * 6.5);

TFLI2C tflI2C;
int16_t  tf_leftDist = 0;    // distance in centimeters
int16_t  tf_leftAddr = 0x10;

int16_t  tf_rightDist = 0;    // distance in centimeters
int16_t  tf_rightAddr = 0x11;

int current_heading = 0;

int n = 6;
int arm_encoders = 2;

int inverter_pin = PA1;
int fire_pin = PA2;

int slp[] = {PC10, PF3, PC6, PB1, PE7, PC2};
int pwr[] = {PC8, PC9, PA6, PA7, PE14, PB0};
int dir[] = {PC11, PC12, PB5, PB3, PE8, PE9};

// int enca[] = {PD1, PD3, PD5, PD7, PF12, PD14}; // interrupt
// int encb[] = {PD0, PD4, PD6, PD2, PF13, PD15};

// int encA[] = {PD9, PE11};
// int encB[] = {PE10, PD8};

int enca[] = {PE10, PD8, PF13, PD15, PD1, PD3}; // interrupt
int encb[] = {PD9, PE11, PF12, PD14, PD0, PD4};

int encA[] = {PD6, PD2};
int encB[] = {PD5, PD7};

volatile long encoder_pos[] = {0, 0};
volatile byte last_angle[] = {0, 0}; // Variables to store the last state of A and B pins

double target[] = {0, 0, 0, 0, 0, 0};

volatile long actual_enc_position[] = {0, 0, 0, 0, 0, 0};
long eprev[] = {0, 0, 0, 0, 0, 0};
long eintegral[] = {0, 0, 0, 0, 0, 0};
double kp[] = {1.0, 1.0, 1.0, 1.0, 5.0, 5.0};
double kd[] = {0.0006, 0.0006, 0.0006, 0.0006, 0.0006, 0.0006};
double ki[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
long pos[] = {0, 0, 0, 0, 0.0, 0.0};

long prevT = 0;
double w[] = {0, 0, 0, 0, 0, 0};

double timeElapsed = 0.0;
int max_power = 255;

int dribblingActive = 0;
double dribbling_equilibrium = 15;
float k_dribble = 1.414;
float k_lateral = 1;
float k_rotation = 1;
float k_inward = 0.2;

// For servo
// Target angle and speed
int servoPin = PE2;
int currentAngle = 90;   // Initial angle of the servo
int targetAngle = 90;   // Desired angle to move to
int speed = 25;         // Speed in degrees per second 
Servo myServo;

// ===== New Global Variables for Passing Mechanism =====
bool passingModeActive = false;               // Set to true when "P1" is received
const int passActivationDistance = 150;        // Threshold distance in cm
long lateralOffset = 0;                         // Accumulated lateral tick offset (for left/right movement)
const long maxLateralOffset = 4000;             // Maximum allowed lateral offset (tunable)
const long lateralSpeedTicksPerSec = 1000;      // Lateral speed (in ticks per second, tunable)

// New variables to freeze lateral updates when the ball is centered
bool freezeActive = false;
unsigned long freezeStartTime = 0;
const unsigned long freezeDuration_us = 100000; // Freeze for 0.5 seconds
// ======================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mag.begin();

  myServo.attach(servoPin);
  myServo.write(currentAngle);

  // Initialize HMC5883L
  pinMode(inverter_pin, OUTPUT);
  pinMode(fire_pin, OUTPUT);
  digitalWrite(inverter_pin,LOW);
  digitalWrite(fire_pin,LOW);

  for (int i = 0; i < n; i++) {
    pinMode(slp[i], OUTPUT);
    pinMode(pwr[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);
    
    digitalWrite(slp[i], LOW);
    digitalWrite(dir[i], LOW);
    analogWrite(pwr[i], 0);
  }
  
  for (int i = 0; i<arm_encoders;i++){
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }
  // for dribbling encoders
  attachInterrupt(digitalPinToInterrupt(encA[0]), readEncoder<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[0]), readEncoder<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), readEncoder<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[1]), readEncoder<1>, CHANGE);

  // for motor encoders
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder0<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder1<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder2<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder3<3>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder4<4>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder5<5>, RISING);
}

void loop() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    if(str[0] == 'C'){
      digitalWrite(inverter_pin,int(str[1]) - '0');
    } else if(str[0] == 'F'){
      digitalWrite(fire_pin,int(str[1])- '0');
    } else if(str[0] == 'D'){
      dribblingActive = int(str[1]) - '0';
      if(dribblingActive){
        calculateDribbling(w[0],w[1],w[2],w[3],w[4],w[5]);
      } else {
        w[4] = 0;
        w[5] = 0; 
      }
    } else if(str[0] == 'S'){
      int targetAngle = str.substring(1).toInt();
      moveServoToAngle(targetAngle, speed);
    }
    // ----- New Passing Mode Commands -----
    else if(str[0] == 'P'){
      if(str[1] == '1'){
        passingModeActive = true;
      } else if(str[1] == '0'){
        passingModeActive = false;
        lateralOffset = 0;  // Reset lateral offset when passing is disengaged
        freezeActive = false;
      }
    }
    // ---------------------------------------
    else if(isValidData(str.c_str())){
      w[0] = 0;
      w[1] = 0;
      w[2] = 0;
      w[3] = 0;
      
      int ptr = 0;
      for (int i = 0; i < 4; i++) {
        while (str[ptr] != ',') {
          w[i] = 10 * w[i] + str[ptr] - '0';
          ptr++;
        }
        ptr++;
      }

      for (int i = 0; i < 4; i++, ptr++) {
        if (str[ptr] == '1') w[i] = -w[i];
      }
      if(dribblingActive){
        calculateDribbling(w[0],w[1],w[2],w[3],w[4],w[5]);
      }
    }
    // Serial.print("Angular velocities\t");
    // Serial.print(w[0]);
    // Serial.print("\t");
    // Serial.print(w[1]);
    // Serial.print("\t");
    // Serial.print(w[2]);
    // Serial.print("\t");
    // Serial.print(w[3]);
    // Serial.println();
  }

  // time difference
  long long currT = micros();
  double deltaT = ((double)(currT - prevT)) / (1.0e6);
  prevT = currT;

  timeElapsed += deltaT;

  for (int i = 0; i < n; i++) target[i] += w[i] * deltaT;

  // ----- New Passing (Ball Catching) Mechanism -----
  if(passingModeActive) {
    // Get updated lidar data for passing mechanism
    tflI2C.getData(tf_leftDist, tf_leftAddr);
    tflI2C.getData(tf_rightDist, tf_rightAddr);
    
    // Determine if either sensor sees the ball within the activation distance
    bool leftActive = (tf_leftDist > 0 && tf_leftDist < passActivationDistance);
    bool rightActive = (tf_rightDist > 0 && tf_rightDist < passActivationDistance);
    
    // If both lidars detect the ball (ball is centered), freeze lateral updates
    if(leftActive && rightActive) {
      if(!freezeActive) {
        freezeActive = true;
        freezeStartTime = micros();
      }
      // If within the freeze threshold time, do not add any lateral ticks
      
      // Even after the freeze period, when the ball is still centered, no lateral update is applied.
    }
    // If not both active, reset the freeze flag and update lateral offset accordingly
    else if(!freezeActive){
      
      // If only the left lidar sees the ball, move left
      if(leftActive && !rightActive) {
        long tickIncrement = (long)(lateralSpeedTicksPerSec * deltaT);
        if(lateralOffset - tickIncrement < -maxLateralOffset) {
          tickIncrement = lateralOffset + maxLateralOffset; // Prevent exceeding negative limit
        }
        lateralOffset -= tickIncrement;
        target[0] -= tickIncrement;
        target[1] += tickIncrement;
        target[2] += tickIncrement;
        target[3] -= tickIncrement;
      }
      // If only the right lidar sees the ball, move right
      else if(rightActive && !leftActive) {
        long tickIncrement = (long)(lateralSpeedTicksPerSec * deltaT);
        if(lateralOffset + tickIncrement > maxLateralOffset) {
          tickIncrement = maxLateralOffset - lateralOffset;
        }
        lateralOffset += tickIncrement;
        target[0] += tickIncrement;
        target[1] -= tickIncrement;
        target[2] -= tickIncrement;
        target[3] += tickIncrement;
      }
    }
    if(micros() - freezeStartTime > freezeDuration_us) {
        freezeActive = false;
        // Freeze: do nothing (allow the bot to stabilize its position)
      }
  }
  // --------------------------------------------------

  noInterrupts();  // disable interrupts temporarily while reading
  for (int i = 0; i < n; i++) {
    pos[i] = actual_enc_position[i];
  }
  interrupts();  // turn interrupts back on

  for (int i = 0; i < n; i++) {
    long e = target[i] - pos[i];
    double dedt = (e - eprev[i]) / (deltaT);
    eintegral[i] = eintegral[i] + e * deltaT;

    // control signal
    double u = kp[i] * e + kd[i] * dedt + ki[i] * eintegral[i];

    // motor power
    int calculated_pwr = (int)fabs(u);
    if (calculated_pwr > max_power) {
      calculated_pwr = max_power;
    }

    // motor direction
    int calculated_dir = 1;
    if (u < 0) {
      calculated_dir = -1;
    }

    // store previous error
    eprev[i] = e;

    // signal the motor
    if (i == 0) setMotor0(calculated_dir, calculated_pwr, i);
    else if (i == 1) setMotor1(calculated_dir, calculated_pwr, i);
    else if (i == 2) setMotor2(calculated_dir, calculated_pwr, i);
    else if (i == 3) setMotor3(calculated_dir, calculated_pwr, i);
    else if (i == 4) setMotor4(calculated_dir, calculated_pwr, i);
    else if (i == 5) setMotor5(calculated_dir, calculated_pwr, i);
  }
  if(dribblingActive){ 
    setDribbling(deltaT);
  }
  // for reading lidar
  // tflI2C.getData( tf_leftDist, tf_leftAddr); // If read okay...
  // tflI2C.getData( tf_rightDist, tf_rightAddr); // If read okay...

  // for reading compass
  // current_heading = getHeading();

  // Serial.print("target:achieved \t");
  // Serial.print(int(target[0]));
  // Serial.print(":");
  // Serial.print(actual_enc_position[0]);
  // Serial.print("  ,  ");
  // Serial.print(int(target[1]));
  // Serial.print(":");
  // Serial.print(actual_enc_position[1]);
  // Serial.print("  ,  ");
  // Serial.print(int(target[2]));
  // Serial.print(":");
  // Serial.print(actual_enc_position[2]);
  // Serial.print("  ,  ");
  // Serial.print(int(target[3]));
  // Serial.print(":");
  // Serial.print(actual_enc_position[3]);
  // Serial.print("  ,  ");
  // Serial.print(int(target[4]));
  // Serial.print(":");
  Serial.print(actual_enc_position[4]);
  Serial.print("  ,  ");
  Serial.print(int(target[5]));
  Serial.print(":");
  Serial.print(actual_enc_position[5]);
  Serial.print("  ,  ");
  Serial.print("arm = ");
  Serial.print(encoder_pos[0]);
  Serial.print(",");
  Serial.print(encoder_pos[1]);
  Serial.print("   ,   lidars: L=");
  Serial.print(tf_leftDist);
  Serial.print(" , R=");
  Serial.print(tf_rightDist);
  Serial.print("  ,  heading = ");
  Serial.print(current_heading);
  Serial.print("\n");
}

void moveServoToAngle(int target, int speed) {
  // Calculate the delay between steps based on speed
  int delayTime = 1000 / speed; // Time per degree (ms)
  
  // Move the servo incrementally
  while (currentAngle != target) {
    if (currentAngle < target) {
      currentAngle++; // Increment angle
    } else if (currentAngle > target) {
      currentAngle--; // Decrement angle
    }
    
    // Write the current angle to the servo
    myServo.write(currentAngle);
    
    // Delay to control speed
    delay(delayTime);
  }
}

void calculateDribbling(double w0, double w1, double w2, double w3, double &w4, double &w5) {
  // Conversion factor: wheel circumference divided by ticks per revolution.
  // Wheel radius = 10 cm, so circumference = 2 * PI * 10.
  
  // Convert the raw wheel tick velocities to linear speeds in cm/s.
  float w0_cm = w0 * tick_to_cm;
  float w1_cm = w1 * tick_to_cm;
  float w2_cm = w2 * tick_to_cm;
  float w3_cm = w3 * tick_to_cm;

  // Compute the translational velocities in cm/s.
  float vx = (w0_cm + w1_cm + w2_cm + w3_cm) / 4.0;
  float vy = (-w0_cm + w1_cm + w2_cm - w3_cm) / 4.0;

  // Compute a rotational component as a linear (tangential) speed in cm/s.
  // (This value represents the difference in wheel speeds that would cause rotation.)
  float omega_lin = (-w0_cm - w1_cm + w2_cm + w3_cm) / 4.0;

  // To get an angular velocity (in rad/s) from the tangential speed,
  // use the effective radius of the robot (assumed BOT_SIZE is the full diameter).
  float r = BOT_SIZE / 2.0;           // effective radius in cm
  float omega_rad = omega_lin / r;    // angular velocity in rad/s

  // Now, using the centripetal formula v = r * omega_rad,
  // we can compute the linear (tangential) velocity due to rotation.
  // (This will effectively be equal to omega_lin, but it makes the concept explicit.)
  float v_c = r * omega_rad;         

  // Incorporate the rotational (centripetal) effect into the effective translational speeds.
  // The coefficients (0.5 for x and 1.0 for y) can be tuned based on experimental behavior.
  float vx_eff = cm_to_tick*(vx);
  float vy_eff = cm_to_tick*(vy + 1.0 * v_c);

  // Finally, calculate the target speeds for the dribbler wheels.
  // Here, k_dribble, k_lateral, and k_inward are tunable gains.
  // We use the absolute angular speed (in rad/s) for the correction term.
  w4 = k_dribble * vx_eff + k_lateral * vy_eff;
  w5 = k_dribble * vx_eff - k_lateral * vy_eff; 
}

void setDribbling(double deltaT){
  float error1 = (abs(encoder_pos[0]) - dribbling_equilibrium)/dribbling_equilibrium;
  float error2 = (abs(encoder_pos[1]) - dribbling_equilibrium)/dribbling_equilibrium;
  float meanError = (error1+error2)/4;
  // error1 += meanError;
  // error2 += meanError;
  target[4] += max(error1*deltaT*1500,-deltaT*1500)*(error1<0);
  target[5] += max(error2*deltaT*1500,-deltaT*1500)*(error2<0);
  
  // int error4 = ((abs(encoder_pos[0]) - dribbling_equilibrium)/dribbling_equilibrium)*255;
  // int error5 = ((abs(encoder_pos[1]) - dribbling_equilibrium)/dribbling_equilibrium)*255;

  // setMotor4(error4>0, abs(error4), 4);
  // setMotor5(error5>0, abs(error5), 5);
}

float getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
 
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  return headingDegrees;
}

bool isInteger(const char *str) {
    if (*str == '-' || *str == '+') str++; // Allow optional sign
    if (!*str) return false; // Empty string after sign is not valid
    while (*str) {
        if (!isdigit(*str)) return false;
        str++;
    }
    return true;
}

bool isValidData(const char *data) {
    const int MAX_LENGTH = 50; // Adjust if needed
    char temp[MAX_LENGTH];
    strncpy(temp, data, MAX_LENGTH - 1); // Make a copy of the input
    temp[MAX_LENGTH - 1] = '\0';         // Ensure null-termination

    int commaCount = 0;
    for (int i = 0; temp[i] != '\0'; i++) {
        if (temp[i] == ',') commaCount++;
    }

    // Check if there are exactly 4 commas
    if (commaCount != 4) return false;

    // Split the string into parts using commas
    char *token = strtok(temp, ",");
    int tokenCount = 0;

    while (token != NULL) {
        tokenCount++;
        if (tokenCount <= 4) {
            // First 4 parts must be integers
            if (!isInteger(token)) return false;
        } else {
            // Last part must be exactly 4 digits (0-9)
            int len = strlen(token);
            if (len != 4) return false;
            for (int i = 0; i < 4; i++) {
                if (!isdigit(token[i])) return false;
            }
        }
        token = strtok(NULL, ",");
    }

    // Ensure there are exactly 5 parts (4 numbers + 1 four-digit number)
    return (tokenCount == 5);
}

void setMotor0(int calculated_dir, int calculated_pwr, int motor_idx) {
  analogWrite(pwr[motor_idx], calculated_pwr);
  if (calculated_dir == 1) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);
}

void setMotor1(int calculated_dir, int calculated_pwr, int motor_idx) {
  analogWrite(pwr[motor_idx], calculated_pwr);
  if (calculated_dir == 1) digitalWrite(dir[motor_idx], LOW);
  else digitalWrite(dir[motor_idx], HIGH);
}

void setMotor2(int calculated_dir, int calculated_pwr, int motor_idx) {
  analogWrite(pwr[motor_idx], calculated_pwr);
  if (calculated_dir == 1) digitalWrite(dir[motor_idx], LOW);
  else digitalWrite(dir[motor_idx], HIGH);
}

void setMotor3(int calculated_dir, int calculated_pwr, int motor_idx) {
  analogWrite(pwr[motor_idx], calculated_pwr);
  if (calculated_dir == 1) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);
}

void setMotor4(int calculated_dir, int calculated_pwr, int motor_idx) {
  analogWrite(pwr[motor_idx], calculated_pwr);
  if (calculated_dir == 1) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);
}

void setMotor5(int calculated_dir, int calculated_pwr, int motor_idx) {
  analogWrite(pwr[motor_idx], calculated_pwr);
  if (calculated_dir == 1) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);
}

template<int j>
void readEncoder0() {
  int b = digitalRead(encb[j]);
  if (b > 0) actual_enc_position[j]++;
  else actual_enc_position[j]--;
}

template<int j>
void readEncoder1() {
  int b = digitalRead(encb[j]);
  if (b > 0) actual_enc_position[j]++;
  else actual_enc_position[j]--;
}

template<int j>
void readEncoder2() {
  int b = digitalRead(encb[j]);
  if (b > 0) actual_enc_position[j]--;
  else actual_enc_position[j]++;
}

template<int j>
void readEncoder3() {
  int b = digitalRead(encb[j]);
  if (b > 0) actual_enc_position[j]--;
  else actual_enc_position[j]++;
}

template<int j>
void readEncoder4() {
  int b = digitalRead(encb[j]);
  if (b > 0) actual_enc_position[j]--;
  else actual_enc_position[j]++;
}

template<int j>
void readEncoder5() {
  int b = digitalRead(encb[j]);
  if (b > 0) actual_enc_position[j]++;
  else actual_enc_position[j]--;
}

template<int j>
void readEncoder() {
  byte MSB = digitalRead(encA[j]);
  byte LSB = digitalRead(encB[j]);
  byte encoded = (MSB << 1) | LSB;
  byte sum = (last_angle[j] << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder_pos[j]++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder_pos[j]--;

  last_angle[j] = encoded;
}