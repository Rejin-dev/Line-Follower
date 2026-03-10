#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- Pin Definitions ---

const int AIN1 = 13;
const int AIN2 = 14;
const int PWMA = 26;

const int BIN1 = 12;
const int BIN2 = 27;
const int PWMB = 25;
const int STBY = 33;

const int START_BUTTON = 21;
const int BLACK_BUTTON = 4;
const int WHITE_BUTTON = 15;
const int ONBOARD_LED = 2;
const int MUX_SIG = 34;
const int S0 = 16;
const int S1 = 5;
const int S2 = 18;
const int S3 = 19;

// --- PID & Speed Variables ---
float Kp = 30.0, Ki = 0.0, Kd = 0.0;
int BASE_SPEED = 150, MAX_SPEED = 255;
// --- Calibration Arrays ---

int sensorThresholds[12];
int blackValues[12];
int whiteValues[12];

// --- Logic Variables ---
float error = 0, lastError = 0, integral = 0;
bool motorRunning = false;

// --- Timer Variables for Finish Line ---
unsigned long runStartTime = 0;
const unsigned long ignoreStartTime = 2000;  // Ignore finish line for first 2000ms (2s)
unsigned long blackDetectStartTime = 0;
const unsigned long requiredBlackTime = 100;  // Must see full black continuously for 100ms
bool isTimingBlack = false;

unsigned long lastDebounceTime = 0;
const int debounceDelay = 200;
int runtime = 0;
bool trackFinished = false;

// --- Streaming Variables ---
unsigned long lastStreamTime = 0;
const int streamInterval = 100; // Streams data every 100ms to avoid Bluetooth flooding

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_PID_Pro");

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  // Button Setups
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(BLACK_BUTTON, INPUT_PULLUP);
  pinMode(WHITE_BUTTON, INPUT_PULLUP);

  for (int i = 0; i < 12; i++)
    sensorThresholds[i] = 1500;

  digitalWrite(STBY, HIGH);
  SerialBT.println("System Ready.");
}

void loop() {
  checkButtons();
  handleBluetooth();
  streamSensorData(); // Continuously streams data without a trigger

  if (motorRunning) {
    calculatePID();
  } else {
    driveMotors(0, 0);
    integral = 0;
    lastError = 0;
  }
}

// --- New Streaming Function ---
void streamSensorData() {
  if (millis() - lastStreamTime >= streamInterval) {
    lastStreamTime = millis();
    String dataStr = "SENSORS:";
    for (int i = 0; i < 12; i++) {
      dataStr += String(readMux(i));
      if (i < 11) dataStr += ",";
    }
    SerialBT.println(dataStr);
  }
}

// --- New Calibration Functions ---

void calibrateBlack() {
  SerialBT.println(">>> STARTING BLACK CALIBRATION in 2s...");
  delay(2000);  // Give user time to position and remove hand

  for (int i = 0; i < 12; i++)
    blackValues[i] = readMux(i);

  updateThresholds();
  SerialBT.println(">>> BLACK CALIBRATION COMPLETE. You can move the bot.");
  // Light up the onboard LED for 0.5 second
  digitalWrite(ONBOARD_LED, HIGH);
  delay(500);
  digitalWrite(ONBOARD_LED, LOW);
}

void calibrateWhite() {
  SerialBT.println(">>> STARTING WHITE CALIBRATION in 2s...");
  delay(2000);

  for (int i = 0; i < 12; i++)
    whiteValues[i] = readMux(i);

  updateThresholds();
  SerialBT.println(">>> WHITE CALIBRATION COMPLETE. You can move the bot.");
  // Light up the onboard LED for 0.5 second
  digitalWrite(ONBOARD_LED, HIGH);
  delay(500);
  digitalWrite(ONBOARD_LED, LOW);
}

// ---------------------------------
void handleCommand(String cmd) {

  if (cmd.startsWith("KP=")) {
    Kp = cmd.substring(3).toFloat();
    SerialBT.println("ACK:KP=" + String(Kp));
  }

  else if (cmd.startsWith("KI=")) {
    Ki = cmd.substring(3).toFloat();
    SerialBT.println("ACK:KI=" + String(Ki));
  }

  else if (cmd.startsWith("KD=")) {
    Kd = cmd.substring(3).toFloat();
    SerialBT.println("ACK:KD=" + String(Kd));
  }

  else if (cmd.startsWith("BASE=")) {
    BASE_SPEED = cmd.substring(5).toInt();
    SerialBT.println("ACK:BASE=" + String(BASE_SPEED));
  }

  else if (cmd.startsWith("MAX=")) {
    MAX_SPEED = cmd.substring(4).toInt();
    SerialBT.println("ACK:MAX=" + String(MAX_SPEED));
  }

  else if (cmd == "RUN=1") {
    startRun();
    SerialBT.println("ACK:RUN=1");
  }

  else if (cmd == "RUN=0") {
    stopRun();
    SerialBT.println("ACK:RUN=0");
  }

  else if (cmd == "CAL=BLACK") {
    calibrateBlack();
    SerialBT.println("ACK:CAL=BLACK");
  }

  else if (cmd == "CAL=WHITE") {
    calibrateWhite();
    SerialBT.println("ACK:CAL=WHITE");
  }

  else if (cmd == "TIME?") {
    SerialBT.println("TIME=" + String(runtime));
  }

  else {
    SerialBT.println("ERROR:UNKNOWN_CMD");
  }
}

void handleBluetooth() {
  if (SerialBT.available()) {

    String command = SerialBT.readStringUntil('\n');
    command.trim();

    handleCommand(command);
  }
}

void updateThresholds() {
  for (int i = 0; i < 12; i++) {
    sensorThresholds[i] = (blackValues[i] + whiteValues[i]) / 2;
  }
}

void calculatePID() {
  float avgPos = 0;
  int count = 0;
  for (int i = 0; i < 12; i++) {
    if (readMux(i) > sensorThresholds[i]) {
      avgPos += (i - 5.5);
      count++;
    }
  }

  // --- Continuous Finish Line Detection ---
  // ==========================================

  // Check if the initial 2-second ignore window has passed
  if (millis() - runStartTime > ignoreStartTime) {
    // If 10 or more sensors see black (Using 10 prevents false positives on curves)
    if (count >= 10) {
      if (!isTimingBlack) {
        // We just hit full black. Start the stopwatch.

        isTimingBlack = true;
        blackDetectStartTime = millis();
      } else if (millis() - blackDetectStartTime > requiredBlackTime) {
        // We have been on full black continuously for over 100ms! It's the finish.
        trackFinished = true;
        runtime = millis() - runStartTime;  // Calculate elapsed time
        
        // *** KEY CHANGE: Automatically stop the robot and notify the app ***
        motorRunning = false;
        driveMotors(0, 0);
        SerialBT.println("TRACK_FINISHED");
        
        return;  // Exit the function immediately
      }
    }

    else {
      // The moment the count drops below 10, reset the stopwatch.
      isTimingBlack = false;
    }
  }
  // ==========================================

  if (count > 0) {
    error = avgPos / count;
    integral = constrain(integral + error, -100, 100);
    float derivative = error - lastError;
    int adjustment = (int)(error * Kp + integral * Ki + derivative * Kd);
    lastError = error;

    driveMotors(constrain(BASE_SPEED + adjustment, 0, MAX_SPEED),
                constrain(BASE_SPEED - adjustment, 0, MAX_SPEED));
  } else {
    // The bot has lost the line (count == 0).
    // Use a controlled search speed to prevent overshooting the line.
    int searchSpeed = BASE_SPEED;

    if (lastError > 0) {
      // Line was last seen on the right. Hard spin right.
      driveMotors(searchSpeed, -searchSpeed);
    } else if (lastError < 0) {
      // Line was last seen on the left. Hard spin left.
      driveMotors(-searchSpeed, searchSpeed);
    } else {
      // If it never saw the line at all (lastError is exactly 0), just stop.
      driveMotors(0, 0);
    }
  }
}

void driveMotors(int left, int right) {
  digitalWrite(AIN1, left >= 0);
  digitalWrite(AIN2, left < 0);
  analogWrite(PWMA, abs(left));

  digitalWrite(BIN1, right >= 0);
  digitalWrite(BIN2, right < 0);
  analogWrite(PWMB, abs(right));
}

int readMux(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  delayMicroseconds(20);
  return analogRead(MUX_SIG);
}

void startRun() {
  motorRunning = true;

  runStartTime = millis();
  isTimingBlack = false;
  integral = 0;
  lastError = 0;
  runtime = 0;  // Reset runtime to 0 at start
  trackFinished = false;

  SerialBT.println("Robot Started");
}

void stopRun() {
  motorRunning = false;
  driveMotors(0, 0);
  integral = 0;
  lastError = 0;

  // Only send "Robot Stopped" if it was a manual stop (not auto-finish)
  if (!trackFinished) {
    runtime = 0;  // Manual stop, no time recorded
    SerialBT.println("Robot Stopped");
  }
}


// --- Updated Button Checking Function ---
void checkButtons() {
  // Run/Stop Button
  if (digitalRead(START_BUTTON) == LOW) {
    delay(500);
    if (millis() - lastDebounceTime > debounceDelay) {
      if (!motorRunning)
        startRun();
      else
        stopRun();


      lastDebounceTime = millis();
    }
  }

  // Black Calibration Button
  if (digitalRead(BLACK_BUTTON) == LOW) {
    if (millis() - lastDebounceTime > debounceDelay) {
      motorRunning = false;  // Optional: Stop motors before calibrating
      calibrateBlack();
      lastDebounceTime = millis();
    }
  }

  // White Calibration Button
  if (digitalRead(WHITE_BUTTON) == LOW) {
    if (millis() - lastDebounceTime > debounceDelay) {
      motorRunning = false;  // Optional: Stop motors before calibrating
      calibrateWhite();
      lastDebounceTime = millis();
    }
  }
}
