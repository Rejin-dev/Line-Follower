// PID logic goes here
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

  if (motorRunning) {
    calculatePID();
  } else {
    driveMotors(0, 0);
    integral = 0;
    lastError = 0;
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

void handleBluetooth() {
  if (SerialBT.available()) {
    
    char type = SerialBT.read();

    if (type == 'b') {
      calibrateBlack();
    } else if (type == 'w') {
      calibrateWhite();
    } else {
      float value = SerialBT.parseFloat();

      switch (type) {
        case 'p':
          Kp = value;
          break;
        case 'i':
          Ki = value;
          break;
        case 'd':
          Kd = value;
          break;
        case 's':
          BASE_SPEED = (int)value;
          break;
        case 'm':
          MAX_SPEED = (int)value;
          break;
        case 't':
          if (trackFinished) {
          SerialBT.printf("Finished the track within:%d\n", (runtime));
          runtime = 0;
          trackFinished = false;
    }

      }

      SerialBT.printf("Updated: P:%.2f I:%.2f D:%.2f Spd:%d\n", Kp, Ki, Kd, BASE_SPEED);
    }
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
        motorRunning = false;
        driveMotors(0, 0);
        trackFinished = true;
        runtime = millis() - runtime;
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

// --- Updated Button Checking Function ---
void checkButtons() {
  // Run/Stop Button
  if (digitalRead(START_BUTTON) == LOW) {
    delay(500);
    if (millis() - lastDebounceTime > debounceDelay) {
      motorRunning = !motorRunning;

      //Reset variables on a fresh start ---
      if (motorRunning) {
        runStartTime = millis();
        isTimingBlack = false;
        integral = 0;
        lastError = 0;
        runtime = millis();
        trackFinished = false;



      }

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
