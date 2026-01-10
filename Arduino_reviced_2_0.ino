#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

// =========================
// CONFIG
// =========================
// Enable = ignore serial & drive motors from DEBUG_Setpoint[]
#define DEBUG_TEST_MODE 0 // Set this to 1 to test with your defined DEBUG values

double DEBUG_Setpoint[4] = {10, 10, 10, 0};
bool   DEBUG_Direction[4] = {0, 0, 1, 1};  // 0 = fwd, 1 = rev

// =========================
// SERIAL FORMAT
// Output format: [FL,FR,RL,RR]\n
// =========================

// =========================
// CONSTANTS & PIN MAP
// =========================
const double pi = 3.14159265359;

const double pulsesPerRevolution = 330 * 4;
const double radPerPulse = (2.0 * pi) / pulsesPerRevolution;

const int PWM_Pins[4]  = {6, 4, 7, 5};
const int DIR1_Pins[4] = {44, 52, 42, 50};
const int DIR2_Pins[4] = {45, 53, 43, 51};

// Encoder pins
Encoder encoders[4] = {
  Encoder(20, 30),
  Encoder(19, 34),
  Encoder(21, 32),
  Encoder(2, 3)
};

// =========================
// PID configuration
// =========================
double Kp = 18.0;
double Ki = 7.0;
double Kd = 0.2;

double Setpoint[4] = {0,0,0,0};
double Input[4] = {0,0,0,0};
double Output[4] = {0,0,0,0};

double filteredInput[4] = {0,0,0,0};
double currentSpeed[4] = {0,0,0,0};

PID pid[4] = {
  PID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT),
  PID(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT),
  PID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT),
  PID(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT)
};

volatile long lastCount[4] = {0,0,0,0};
volatile long encCount[4] = {0,0,0,0};

// Encoders updated in ISR
void ISR0() { encCount[0] = encoders[0].read(); }
void ISR1() { encCount[1] = encoders[1].read(); }
void ISR2() { encCount[2] = encoders[2].read(); }
void ISR3() { encCount[3] = encoders[3].read(); }

// =========================
// MOTOR DRIVE
// =========================
void stopMotor(int i) {
  digitalWrite(DIR1_Pins[i], LOW);
  digitalWrite(DIR2_Pins[i], LOW);
  analogWrite(PWM_Pins[i], 0);
}

void driveMotor(int i) {

  if (fabs(Setpoint[i]) < 0.1) {
    pid[i].SetMode(MANUAL);   // freeze PID
    pid[i].SetMode(AUTOMATIC); // restart fresh integral
    Output[i] = 0; 
    stopMotor(i);
    return;
  }

  pid[i].Compute();

  // Output[i] is now signed (rad/s). We scale it to PWM (0-255).
  // Calculate the scaling factor: (255.0 / 33.0) 
  const double SF = 255.0 / 33.0;
  
  // The signed_pwm will range from approx -255 to 255
  double signed_pwm = Output[i] * SF;

  // PWM is the magnitude of the calculated power
  int PWM = constrain(abs(signed_pwm), 0, 255);

  if (PWM < 30) {
    stopMotor(i);
    return;
  }
  
  // Direction is determined by the sign of the PID's *output*
  // If signed_pwm is negative (less than 0), dir is 1 (reverse).
  bool dir = signed_pwm < 0; 

  digitalWrite(DIR1_Pins[i], dir);
  digitalWrite(DIR2_Pins[i], !dir);
  analogWrite(PWM_Pins[i], PWM);
}



// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(PWM_Pins[i], OUTPUT);
    pinMode(DIR1_Pins[i], OUTPUT);
    pinMode(DIR2_Pins[i], OUTPUT);
    pid[i].SetMode(AUTOMATIC);
    pid[i].SetControllerDirection(DIRECT);
    
    // FIX 3: Set signed output limits (in units of rad/s)
    pid[i].SetOutputLimits(-33.0, 33.0);
  }

  attachInterrupt(digitalPinToInterrupt(20), ISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), ISR3, CHANGE);
}



// =========================
// LOOP
// =========================
unsigned long lastUpdate = 0;
const int updateInterval = 5;   // Changed to 5ms (200Hz) to match control loop frequency

// unsigned long lastPrintTime = 0;
// const int PRINT_INTERVAL_MS = 20; // 50 Hz

void loop() {

  // ===========================================
  // DEBUG MODE OVERRIDE
  // ===========================================
  if (DEBUG_TEST_MODE) {
    for (int i=0;i<4;i++) {
      // Setpoint is now signed.
      Setpoint[i] = DEBUG_Setpoint[i] * (DEBUG_Direction[i] ? -1.0 : 1.0);
    }
  }
  else {
    // Normal mode: listen for commands like:
    // [1.5,0.0,1.0,-1.0] - these are already signed
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.startsWith("[") && s.endsWith("]")) {
        s = s.substring(1, s.length()-1);

        int idx = 0;
        char *token = strtok((char*)s.c_str(), ",");
        while (token && idx < 4) {
          Setpoint[idx] = atof(token);
          token = strtok(NULL, ",");
          idx++;
        }
      }
    }
  }


  // ===========================================
  // 200 Hz control & encoder processing
  // ===========================================
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    for (int i = 0; i < 4; i++) {

      long countNow = encCount[i];
      long delta = countNow - lastCount[i];
      lastCount[i] = countNow;

      double dt = updateInterval / 1000.0;

      double radPerSec = (delta / dt) * radPerPulse;
      
      // FIX 1: Reverse the sign of the calculated speed (Encoder Polarity)
      radPerSec = -radPerSec; 

      // Low-pass filter
      filteredInput[i] += (radPerSec - filteredInput[i]) * 0.05;

      currentSpeed[i] = filteredInput[i];

      // FIX 2: Input[i] must be the signed speed for the PID to work.
      Input[i] = filteredInput[i]; 

      driveMotor(i);
    }

    // ===========================================
    // SEND SPEED FEEDBACK IN BRACKET FORMAT
    // ===========================================

    Serial.print("[");
    Serial.print(currentSpeed[0], 2); Serial.print(",");
    Serial.print(currentSpeed[1], 2); Serial.print(",");
    Serial.print(currentSpeed[2], 2); Serial.print(",");
    Serial.print(currentSpeed[3], 2);
    Serial.println("]");
  }
}
