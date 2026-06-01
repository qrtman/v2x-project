/**
 * Project Antigravity: Software-Defined Vehicle (SDV) Prototype
 * Fully Autonomous Warehouse AGV + AEB Safety Integration Firmware
 * 
 * Hardware Pin Mapping:
 * - Left Motors PWM (ENA): Pin 5 (Remove Jumper!)
 * - Left Direction 1 (IN1): Pin 6
 * - Left Direction 2 (IN2): Pin 7
 * - Right Direction 1 (IN3): Pin 8
 * - Right Direction 2 (IN4): Pin 4
 * - Right Motors PWM (ENB): Pin 3 (Remove Jumper!)
 * - SG90 Micro Servo: Pin 9
 * - Ultrasonic Trig: Pin 11
 * - Ultrasonic Echo: Pin 12
 * - HW-096 Tracing Module Channel 1 (Right): Pin A0
 * - HW-096 Tracing Module Channel 2 (Mid-R): Pin A1
 * - HW-096 Tracing Module Channel 3 (Mid-L): Pin A2
 * - HW-096 Tracing Module Channel 4 (Left):  Pin A3
 * - Hardware Serial: Pins 0 & 1 (9600 Baud for Telemetry & Mission triggers)
 * 
 * Note: Since this setup bypasses the standard L293D shield in favor of L298N discrete 
 * control, it maps raw PWM registers directly, ensuring high precision.
 */

#include <Servo.h>

// --- PIN DEFINITIONS ---
const int PIN_ENA = 5;  // Left Motor PWM Speed Control
const int PIN_IN1 = 6;  // Left Motor Direction 1
const int PIN_IN2 = 7;  // Left Motor Direction 2
const int PIN_IN3 = 8;  // Right Motor Direction 1
const int PIN_IN4 = 4;  // Right Motor Direction 2
const int PIN_ENB = 3;  // Right Motor PWM Speed Control

const int PIN_SERVO = 9;   // SG90 Micro Servo Pin
const int PIN_TRIG  = 11;  // HC-SR04 Ultrasonic Trigger
const int PIN_ECHO  = 12;  // HC-SR04 Ultrasonic Echo

// HW-096 Tracing Sensor Pins (A0-A3 Sequential Layout)
const int PIN_TRACK_R  = A0; // Right-most sensor (Channel 1)
const int PIN_TRACK_MR = A1; // Middle-Right sensor (Channel 2)
const int PIN_TRACK_ML = A2; // Middle-Left sensor (Channel 3)
const int PIN_TRACK_L  = A3; // Left-most sensor (Channel 4)

// --- CONFIGURATION CONSTANTS ---
const float TRACK_WIDTH_B = 15.5;      // Base width in cm
const float V_SUPPLY = 12.0;           // Battery supply voltage
const float MAX_PHYSICAL_SPEED = 50.0; // Max speed at 12V in cm/s
const int AEB_THRESHOLD_CM = 20;       // Safety braking threshold

// --- VEHICLE VELOCITY PROFILES ---
const int BASE_DRIVE_SPEED = 180;      // Base speed for straight line following (0-255)
const int TURN_SPEED = 150;            // Pivot/steer motor speed

// --- STATE MANAGEMENT ---
Servo radarServo;
int currentServoAngle = 90;
bool sweepingUp = true;
unsigned long lastServoTime = 0;
const unsigned long SERVO_INTERVAL_MS = 20; // 20ms per degree sweep

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 500; // Telemetry reports every 500ms

// AGV Mission States
// 'S' = Safe Stop (Waiting for mission start)
// 'A' = Autonomous Line Following (Active AGV mission)
// 'E' = Emergency Scan / Halt (Obstacle detected, checking width)
// 'B' = Evading - Backing Up (Runs for 1.5 seconds)
// 'T' = Evading - Turning Right (Runs for 1.2 seconds)
char currentMissionState = 'S'; 
bool isObstacleDetected = false;
float obstacleDistanceCm = 999.0;

// Evasion sequence timing tracker
unsigned long evasionStartTime = 0;

// Sector distance memory for 180-degree wide closure detection
float distLeftSector = 999.0;
float distCenterSector = 999.0;
float distRightSector = 999.0;

// Emergency quick scan variables
int quickScanStage = 0; // 0 = start, 1 = swept up, 2 = swept down (finished)
int consecutiveCloseTicks = 0; // Tracks current contiguous close readings
int maxConsecutiveCloseTicks = 0; // Tracks maximum consecutive close span

// Motor tracking speeds for kinematics telemetry
int leftMotorPwm = 0;
int rightMotorPwm = 0;
char motorDirectionCode = 'S'; // 'F'=Forward, 'B'=Backward, 'L'=Left, 'R'=Right, 'S'=Stop

void setup() {
  // Motor Outputs
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  
  // Ultrasonic Pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Tracing Pins
  pinMode(PIN_TRACK_R,  INPUT);
  pinMode(PIN_TRACK_MR, INPUT);
  pinMode(PIN_TRACK_ML, INPUT);
  pinMode(PIN_TRACK_L,  INPUT);

  // Initialize Servo
  radarServo.attach(PIN_SERVO);
  radarServo.write(currentServoAngle);

  // Initialize Serial Telemetry Port
  Serial.begin(9600);

  // Apply absolute safe stop state at start
  haltMotors();

  // 5-second floor-mount warm-up delay
  // Gives you time to set the vehicle centered on your line track before autonomous drive begins
  delay(5000);
  currentMissionState = 'A'; // Automatically start the AGV mission
}

void loop() {
  // 1. High-Frequency Range Acquisition
  measureDistance();

  // 2. Safety Planner Loop (Autonomous Emergency Braking - AEB & Emergency Scan Trigger)
  if (obstacleDistanceCm > 0.0 && obstacleDistanceCm <= AEB_THRESHOLD_CM) {
    if (currentMissionState == 'A') { // Only trigger emergency scan if we were actively following the line
      currentMissionState = 'E'; // Switch to Quick Emergency Scan state
      quickScanStage = 0;
      consecutiveCloseTicks = 0;
      maxConsecutiveCloseTicks = 0;
      currentServoAngle = 30; // Force starting scan angle
      sweepingUp = true;
      isObstacleDetected = true;
      haltMotors(); // Full stop instantly
      
      // Clear old sector readings to map fresh data
      distLeftSector = 999.0;
      distCenterSector = 999.0;
      distRightSector = 999.0;

      Serial.print(F("{\"alert\":\"EMERGENCY_HALT_QUICK_SCAN\",\"distance\":"));
      Serial.print(obstacleDistanceCm, 1);
      Serial.println(F("}"));
    }
  }

  // 3. Command Parser (Mission Control interface)
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    // 'F' or 'A' triggers Mission Start (AGV active)
    // 'S' triggers Mission Stop
    if (incomingChar == 'F' || incomingChar == 'A') {
      if (!isObstacleDetected) {
        currentMissionState = 'A'; // Start AGV Mission
      }
    } else if (incomingChar == 'S') {
      currentMissionState = 'S'; // Stop Mission
      haltMotors();
    }
  }

  // 4. Autonomous Control & Evasion State Machine
  if (currentMissionState == 'E') {
    haltMotors(); // Full stop while scanning is active

    if (quickScanStage == 2) {
      // Quick scan completed! Evaluate frontal threat using geometric angular width
      // 12 ticks of consecutive proximity close signals = 24 degrees of wide block.
      // This solves specular reflection loss on flat walls at steep angles!
      bool is180DegreeClosure = (maxConsecutiveCloseTicks >= 12);
      
      if (is180DegreeClosure) {
        // Wide wall detected: Trigger Active Backing Evasion State ('B')
        currentMissionState = 'B';
        evasionStartTime = millis();
      } else {
        // Point/narrow obstacle: check if center path is clear now
        if (distCenterSector > AEB_THRESHOLD_CM) {
          // Clear path: Resume Line-Following!
          currentMissionState = 'A';
          isObstacleDetected = false;
        } else {
          // Center still blocked: restart quick scan
          quickScanStage = 0;
          consecutiveCloseTicks = 0;
          maxConsecutiveCloseTicks = 0;
          currentServoAngle = 30;
          sweepingUp = true;
          distLeftSector = 999.0;
          distCenterSector = 999.0;
          distRightSector = 999.0;
        }
      }
    }
  }
  else if (currentMissionState == 'B') {
    // Stage 1: Go backward to clear the obstacle
    if (millis() - evasionStartTime >= 1500) {
      // 1.5 seconds finished -> transition to Stage 2: Turn Right
      currentMissionState = 'T';
      evasionStartTime = millis();
    } else {
      driveMotorsBackward(BASE_DRIVE_SPEED, BASE_DRIVE_SPEED);
    }
  } 
  else if (currentMissionState == 'T') {
    // Stage 2: Turn Right to steer clear
    if (millis() - evasionStartTime >= 1200) {
      // 1.2 seconds finished -> transition back to normal line-following
      currentMissionState = 'A';
      isObstacleDetected = false;
    } else {
      steerRight(TURN_SPEED);
    }
  } 
  else if (currentMissionState == 'A') {
    // Normal autonomous line-tracking
    runLineTrackingLogic();
  } 
  else {
    // Stopped State ('S')
    haltMotors();
  }

  // 5. Servo Reconnaissance Scan (Non-blocking)
  updateServoPosition();

  // 6. Kinematic Telemetry Streaming (Every 500ms)
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = currentMillis;
    broadcastDiagnostics();
  }
}

/**
 * Sweeps the micro-servo between 30 and 150 degrees.
 */
void updateServoPosition() {
  unsigned long currentMillis = millis();
  
  // Establish speed-dynamic sweep limits
  // When active line-following ('A'), narrow threat cone (75° to 105°) to reduce blind spots.
  // Otherwise, run full 180° sweep (30° to 150°) for complete room mapping.
  int minAngle = (currentMissionState == 'A') ? 75 : 30;
  int maxAngle = (currentMissionState == 'A') ? 105 : 150;
  
  // Quick scan mode is 5x faster (8ms per 2-degree step) than normal sweep (20ms per 1-degree step)
  unsigned long interval = (currentMissionState == 'E') ? 8 : SERVO_INTERVAL_MS;
  int degreeStep = (currentMissionState == 'E') ? 2 : 1;

  // Clamp current angle immediately to stay within dynamic bounds
  if (currentServoAngle < minAngle) currentServoAngle = minAngle;
  if (currentServoAngle > maxAngle) currentServoAngle = maxAngle;

  if (currentMillis - lastServoTime >= interval) {
    lastServoTime = currentMillis;

    if (sweepingUp) {
      currentServoAngle += degreeStep;
      if (currentServoAngle >= maxAngle) {
        currentServoAngle = maxAngle;
        sweepingUp = false;
        if (currentMissionState == 'E') {
          quickScanStage = 1; // Finished sweeping up
        }
      }
    } else {
      currentServoAngle -= degreeStep;
      if (currentServoAngle <= minAngle) {
        currentServoAngle = minAngle;
        sweepingUp = true;
        if (currentMissionState == 'E') {
          quickScanStage = 2; // Finished sweeping down (full scan cycle done!)
        }
      }
    }
    radarServo.write(currentServoAngle);
  }
}

/**
 * Triggers HC-SR04 ultrasonic pulse and measures echo flight duration.
 */
void measureDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 20000); // 20ms timeout
  
  if (duration > 0) {
    obstacleDistanceCm = (duration * 0.0343) / 2.0;
    
    // Update multi-sector memory based on current servo angle to map 180° field of view
    if (currentServoAngle >= 30 && currentServoAngle < 70) {
      distLeftSector = obstacleDistanceCm;
    } else if (currentServoAngle >= 70 && currentServoAngle < 110) {
      distCenterSector = obstacleDistanceCm;
    } else if (currentServoAngle >= 110 && currentServoAngle <= 150) {
      distRightSector = obstacleDistanceCm;
    }

    // Accumulate consecutive close ticks in Emergency Scan mode
    if (currentMissionState == 'E') {
      if (obstacleDistanceCm <= AEB_THRESHOLD_CM) {
        consecutiveCloseTicks++;
        if (consecutiveCloseTicks > maxConsecutiveCloseTicks) {
          maxConsecutiveCloseTicks = consecutiveCloseTicks;
        }
      } else {
        consecutiveCloseTicks = 0;
      }
    }
  } else {
    obstacleDistanceCm = 999.0;
    
    // Reset individual sector memory as the sweep passes clean zones
    if (currentServoAngle >= 30 && currentServoAngle < 70) {
      distLeftSector = 999.0;
    } else if (currentServoAngle >= 70 && currentServoAngle < 110) {
      distCenterSector = 999.0;
    } else if (currentServoAngle >= 110 && currentServoAngle <= 150) {
      distRightSector = 999.0;
    }

    if (currentMissionState == 'E') {
      consecutiveCloseTicks = 0;
    }
  }
}

/**
 * Reads the 4-channel tracking sensors and adjusts H-Bridge pins for differential line following.
 * HIGH (1) = BLACK line detected, LOW (0) = WHITE floor
 */
void runLineTrackingLogic() {
  int r  = digitalRead(PIN_TRACK_R);  // Right-most
  int mr = digitalRead(PIN_TRACK_MR); // Mid-Right
  int ml = digitalRead(PIN_TRACK_ML); // Mid-Left
  int l  = digitalRead(PIN_TRACK_L);  // Left-most

  // Safe Dynamic Velocity Map: scale forward speed based on sensor angle offset from center (90)
  // When looking straight (90°), delta = 0, drive speed is BASE_DRIVE_SPEED (180).
  // When checking side blind spots (75° or 105°), delta = 15, speed decays to 180 - (15 * 2) = 150.
  // This automatically slows the car down to avoid hitting obstacles in its blind spots!
  int deltaAngle = abs(currentServoAngle - 90);
  int dynamicDriveSpeed = BASE_DRIVE_SPEED - (deltaAngle * 2);
  if (dynamicDriveSpeed < 140) dynamicDriveSpeed = 140; // Maintain threshold minimum speed

  // Case 1: Centered on line (middle sensors see black)
  if (ml == HIGH && mr == HIGH) {
    driveMotorsForward(dynamicDriveSpeed, dynamicDriveSpeed);
  }
  // Case 2: Drifting to the right (left-middle or left-most sees black) -> Steer Left
  else if (ml == HIGH || l == HIGH) {
    steerLeft(TURN_SPEED);
  }
  // Case 3: Drifting to the left (right-middle or right-most sees black) -> Steer Right
  else if (mr == HIGH || r == HIGH) {
    steerRight(TURN_SPEED);
  }
  // Case 4: Line lost (all see white) or all see black -> Default to slow forward search
  else {
    driveMotorsForward(100, 100);
  }
}

/**
 * Safe Stop - Cuts all H-Bridge drive power
 */
void haltMotors() {
  leftMotorPwm = 0;
  rightMotorPwm = 0;
  motorDirectionCode = 'S';
  
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

/**
 * Drives both motor pairs backward
 */
void driveMotorsBackward(int leftSpeed, int rightSpeed) {
  leftMotorPwm = leftSpeed;
  rightMotorPwm = rightSpeed;
  motorDirectionCode = 'B';

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  
  analogWrite(PIN_ENA, leftSpeed);
  analogWrite(PIN_ENB, rightSpeed);
}

/**
 * Drives both motor pairs forward
 */
void driveMotorsForward(int leftSpeed, int rightSpeed) {
  leftMotorPwm = leftSpeed;
  rightMotorPwm = rightSpeed;
  motorDirectionCode = 'F';

  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  
  analogWrite(PIN_ENA, leftSpeed);
  analogWrite(PIN_ENB, rightSpeed);
}

/**
 * Steers Left (Left wheels spin backward, Right wheels spin forward)
 */
void steerLeft(int speed) {
  leftMotorPwm = speed;
  rightMotorPwm = speed;
  motorDirectionCode = 'L';

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  
  analogWrite(PIN_ENA, speed);
  analogWrite(PIN_ENB, speed);
}

/**
 * Steers Right (Left wheels spin forward, Right wheels spin backward)
 */
void steerRight(int speed) {
  leftMotorPwm = speed;
  rightMotorPwm = speed;
  motorDirectionCode = 'R';

  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  
  analogWrite(PIN_ENA, speed);
  analogWrite(PIN_ENB, speed);
}

/**
 * Calculates physical kinematic metrics and streams telemetry JSON packet.
 */
void broadcastDiagnostics() {
  // Compute effective motor voltage based on left side active PWM
  float effectiveVoltage = V_SUPPLY * (leftMotorPwm / 255.0);

  // Map theoretical wheel velocities based on direction code
  float vL = 0.0;
  float vR = 0.0;
  float leftScalar = MAX_PHYSICAL_SPEED * (leftMotorPwm / 255.0);
  float rightScalar = MAX_PHYSICAL_SPEED * (rightMotorPwm / 255.0);

  switch (motorDirectionCode) {
    case 'F': vL = leftScalar;  vR = rightScalar;  break;
    case 'L': vL = -leftScalar; vR = rightScalar;  break;
    case 'R': vL = leftScalar;  vR = -rightScalar; break;
    case 'S': vL = 0.0;         vR = 0.0;          break;
  }

  // Linear and Angular formulas
  float linearVelocity = (vR + vL) / 2.0;
  float angularVelocity = (vR - vL) / TRACK_WIDTH_B;

  // Read Tracing status
  int r  = digitalRead(PIN_TRACK_R);
  int mr = digitalRead(PIN_TRACK_MR);
  int ml = digitalRead(PIN_TRACK_ML);
  int l  = digitalRead(PIN_TRACK_L);

  // Stream structured serial data
  Serial.print(F("{\"angle\":"));
  Serial.print(currentServoAngle);
  Serial.print(F(",\"distance\":"));
  Serial.print(obstacleDistanceCm, 1);
  Serial.print(F(",\"v\":"));
  Serial.print(linearVelocity, 2);
  Serial.print(F(",\"w\":"));
  Serial.print(angularVelocity, 2);
  Serial.print(F(",\"v_eff\":"));
  Serial.print(effectiveVoltage, 2);
  Serial.print(F(",\"state\":\""));
  Serial.print(isObstacleDetected ? F("AEB") : (currentMissionState == 'S' ? F("STOP") : F("OBU")));
  Serial.print(F("\",\"cmd\":\""));
  Serial.print(motorDirectionCode);
  Serial.print(F("\",\"trace\":["));
  Serial.print(l);
  Serial.print(F(","));
  Serial.print(ml);
  Serial.print(F(","));
  Serial.print(mr);
  Serial.print(F(","));
  Serial.print(r);
  Serial.println(F("]}"));
}
