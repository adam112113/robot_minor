# Arduino PID Controller Optimization for SLAM

## ⚡ QUICK FIX - Timer Settings

**Change these lines in your Arduino code:**

```cpp
// OLD (WRONG - causes buffer overflow and jitter):
unsigned long lastUpdate = 0;
const int updateInterval = 5;   // 200 Hz - TOO FAST!

// NEW (CORRECT - synchronized with ROS2):
unsigned long lastPIDUpdate = 0;
const int PID_INTERVAL_MS = 10;      // 100 Hz - PID control

unsigned long lastSerialSend = 0;
const int SERIAL_INTERVAL_MS = 20;   // 50 Hz - Serial output
```

**Why this fixes the lag:**
- **Before**: Arduino sent at 200 Hz, but Pi read at 50 Hz → Buffer overflow + timing jitter
- **After**: Arduino sends at 50 Hz matching Pi → Perfect synchronization
- **PID still runs at 100 Hz** for responsive motor control

---

## Complete System Timing Diagram

```
Arduino → Serial @ 50 Hz (every 20ms)
   ↓
Pi Serial Driver reads @ 50 Hz (every 20ms) ✅ SYNCHRONIZED
   ↓  
Odometry calculates @ 50 Hz (every 20ms) ✅ SYNCHRONIZED
   ↓
TF published @ 50 Hz (every 20ms) ✅ SYNCHRONIZED
   ↓
SLAM uses TF @ 50 Hz → Map stays stationary! ✅
```

---

## Problem
Your Arduino sends encoder feedback that affects odometry accuracy for SLAM. 
If the Arduino sends data slowly or has a slow PID loop, it causes the map redraw issue you're seeing.

## What to Check in Your Arduino Code

### 1. Serial Print Rate
**Location:** Main `loop()` function

**Check:**
```cpp
void loop() {
    // How often does this execute?
    // Should be AT LEAST 50 Hz (every 20ms) for good SLAM performance
    
    Serial.print("[");
    Serial.print(wheel_fl);  // Front left angular velocity
    Serial.print(",");
    Serial.print(wheel_fr);  // Front right
    Serial.print(",");
    Serial.print(wheel_rl);  // Rear left  
    Serial.print(",");
    Serial.print(wheel_rr);  // Rear right
    Serial.println("]");
}
```

**Fix:** Add timing control
```cpp
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL_MS = 20;  // 50 Hz

void loop() {
    unsigned long currentTime = millis();
    
    // Your PID and motor control code here...
    
    // Send feedback at fixed 50 Hz rate
    if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = currentTime;
        
        Serial.print("[");
        Serial.print(wheel_fl_velocity, 3);  // 3 decimal places
        Serial.print(",");
        Serial.print(wheel_fr_velocity, 3);
        Serial.print(",");
        Serial.print(wheel_rl_velocity, 3);
        Serial.print(",");
        Serial.print(wheel_rr_velocity, 3);
        Serial.println("]");
    }
}
```

### 2. PID Controller Update Rate
**Check:**
- How often does your PID controller calculate new motor speeds?
- Are encoder readings taken every loop iteration?

**Recommended:**
```cpp
const int PID_UPDATE_MS = 10;  // 100 Hz PID updates
unsigned long lastPIDTime = 0;

void loop() {
    unsigned long now = millis();
    
    // Read encoders every loop (fast!)
    readEncoders();
    
    // Update PID at 100 Hz
    if (now - lastPIDTime >= PID_UPDATE_MS) {
        lastPIDTime = now;
        float dt = PID_UPDATE_MS / 1000.0;  // seconds
        
        // Calculate velocities from encoder deltas
        calculateWheelVelocities(dt);
        
        // Run PID controllers
        updatePIDControllers(dt);
        
        // Apply motor commands
        setMotorSpeeds();
    }
    
    // Send serial feedback at 50 Hz (see above)
    sendSerialFeedback();
}
```

### 3. Encoder Reading
**Check:**
- Are you using interrupts for encoder counting? (BEST)
- Or polling in loop()? (SLOWER)

**Best practice:**
```cpp
volatile long encoder_fl = 0;
volatile long encoder_fr = 0;
volatile long encoder_rl = 0;
volatile long encoder_rr = 0;

void setup() {
    // Attach interrupts for all encoder channels
    attachInterrupt(digitalPinToInterrupt(ENC_FL_A), encoderFL_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_A), encoderFR_ISR, CHANGE);
    // ... etc
}

void encoderFL_ISR() {
    // Count encoder ticks in interrupt
    if (digitalRead(ENC_FL_A) == digitalRead(ENC_FL_B)) {
        encoder_fl++;
    } else {
        encoder_fl--;
    }
}
```

### 4. Velocity Calculation
**Important:** Send actual wheel velocities (rad/s), not encoder counts!

```cpp
long last_encoder_fl = 0;
float wheel_fl_velocity = 0.0;  // rad/s

const int ENCODER_PPR = 360;  // Pulses per revolution
const float WHEEL_RADIUS = 0.03;  // meters

void calculateWheelVelocities(float dt) {
    // Calculate angular velocity for each wheel
    long delta_fl = encoder_fl - last_encoder_fl;
    last_encoder_fl = encoder_fl;
    
    // Convert encoder delta to rad/s
    float revolutions = delta_fl / (float)ENCODER_PPR;
    wheel_fl_velocity = (revolutions * 2.0 * PI) / dt;  // rad/s
    
    // Repeat for other wheels...
}
```

## Quick Diagnostic Test

Add this to your Arduino to measure actual loop rate:

```cpp
unsigned long loopCount = 0;
unsigned long lastStatsTime = 0;

void loop() {
    loopCount++;
    
    // Your existing code...
    
    // Print loop frequency every second
    if (millis() - lastStatsTime >= 1000) {
        Serial.print("Loop Hz: ");
        Serial.println(loopCount);
        loopCount = 0;
        lastStatsTime = millis();
    }
}
```

**Expected output:** Should show 500-10000+ Hz depending on Arduino speed
**Problem if:** Shows <100 Hz (loop is too slow)

## Summary: Target Rates

| Component | Target Rate | Critical? |
|-----------|-------------|-----------|
| Serial feedback to Pi | **50 Hz minimum** | ✅ YES |
| PID controller updates | 50-100 Hz | ✅ YES |
| Encoder reading | Every loop or interrupt | ✅ YES |
| Main loop() | >500 Hz | Recommended |

## Testing After Changes

1. Upload Arduino code
2. On Raspberry Pi, run: `./check_rates.sh`
3. Check `/fb_speed` rate - should be ~50 Hz
4. If still slow, increase Arduino serial send rate to 100 Hz



# Current Arduino Code:

```
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
// LOOP - OPTIMIZED FOR ROS2 SLAM
// =========================

// TIMING CONFIGURATION - CRITICAL FOR SLAM PERFORMANCE
// =====================================================
// PID Update: 100 Hz (10ms) - Fast control loop for responsive motors
// Serial Send: 50 Hz (20ms)  - Matches ROS2 serial_driver read rate
// 
// This prevents:
// - Serial buffer overflow (Arduino sending faster than Pi reads)
// - Timing jitter in odometry
// - TF transform delays

unsigned long lastPIDUpdate = 0;
const int PID_INTERVAL_MS = 10;      // 100 Hz - PID control loop

unsigned long lastSerialSend = 0;
const int SERIAL_INTERVAL_MS = 20;   // 50 Hz - Serial feedback (matches Pi)

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
  // PID CONTROL LOOP - 100 Hz (every 10ms)
  // Fast control for responsive motor performance
  // ===========================================
  unsigned long currentTime = millis();
  
  if (currentTime - lastPIDUpdate >= PID_INTERVAL_MS) {
    lastPIDUpdate = currentTime;

    double dt = PID_INTERVAL_MS / 1000.0;

    for (int i = 0; i < 4; i++) {
      long countNow = encCount[i];
      long delta = countNow - lastCount[i];
      lastCount[i] = countNow;

      double radPerSec = (delta / dt) * radPerPulse;
      
      // FIX 1: Reverse the sign of the calculated speed (Encoder Polarity)
      radPerSec = -radPerSec; 

      // Low-pass filter (alpha = 0.05 for smoothing)
      filteredInput[i] += (radPerSec - filteredInput[i]) * 0.05;

      currentSpeed[i] = filteredInput[i];

      // FIX 2: Input[i] must be the signed speed for the PID to work.
      Input[i] = filteredInput[i]; 

      driveMotor(i);
    }
  }

  // ===========================================
  // SERIAL FEEDBACK - 50 Hz (every 20ms)
  // Synchronized with ROS2 serial_driver read rate
  // ===========================================
  if (currentTime - lastSerialSend >= SERIAL_INTERVAL_MS) {
    lastSerialSend = currentTime;

    Serial.print("[");
    Serial.print(currentSpeed[0], 2); Serial.print(",");
    Serial.print(currentSpeed[1], 2); Serial.print(",");
    Serial.print(currentSpeed[2], 2); Serial.print(",");
    Serial.print(currentSpeed[3], 2);
    Serial.println("]");
  }
}
```