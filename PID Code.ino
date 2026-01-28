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

const int BUTTON_PIN = 21;

const int MUX_SIG = 34;
const int S0 = 16;
const int S1 = 5;
const int S2 = 18;
const int S3 = 19;

// --- PID & Speed Variables ---
float Kp = 10.0, Ki = 0.01, Kd = 0.0;
int BASE_SPEED = 50, MAX_SPEED = 255;
// --- Calibration Arrays ---

int sensorThresholds[12];
int blackValues[12];
int whiteValues[12];

// --- Logic Variables ---
float error = 0, lastError = 0, integral = 0;
bool motorRunning = false;
unsigned long lastDebounceTime = 0;
const int debounceDelay = 200;

void setup()
{

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

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < 12; i++)
    sensorThresholds[i] = 2000;

  digitalWrite(STBY, HIGH);
  SerialBT.println("System Ready.");
  SerialBT.println("Send 'b' for Black, 'w' for White.");
}

void loop()
{

  checkButton();
  handleBluetooth();
  if (motorRunning)
  {
    calculatePID();
  }
  else
  {
    driveMotors(0, 0);
    integral = 0;
    lastError = 0;
  }
}

void handleBluetooth()
{
  if (SerialBT.available())
  {
    char type = SerialBT.read();
    if (type == 'b')
    {
      SerialBT.println(">>> STARTING BLACK CALIBRATION...");
      SerialBT.println("Place all sensors on the black line. Starting in 2s...");
      delay(2000); // Give user time to position and remove hand
      SerialBT.println("Reading now... DO NOT MOVE...");

      for (int i = 0; i < 12; i++)
        blackValues[i] = readMux(i);

      updateThresholds();

      SerialBT.println(">>> BLACK CALIBRATION COMPLETE. You can move the bot.");
    }

    else if (type == 'w')
    {
      SerialBT.println(">>> STARTING WHITE CALIBRATION...");
      SerialBT.println("Place all sensors on white floor. Starting in 2s...");
      delay(2000);

      SerialBT.println("Reading now... DO NOT MOVE...");

      for (int i = 0; i < 12; i++)
        whiteValues[i] = readMux(i);

      updateThresholds();
      SerialBT.println(">>> WHITE CALIBRATION COMPLETE. You can move the bot.");
    }
    else
    {
      float value = SerialBT.parseFloat();

      switch (type)
      {
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
      }

      SerialBT.printf("Updated: P:%.2f I:%.2f D:%.2f Spd:%d\n", Kp, Ki, Kd, BASE_SPEED);
    }
  }
}
void updateThresholds()
{
  for (int i = 0; i < 12; i++)
  {
    sensorThresholds[i] = (blackValues[i] + whiteValues[i]) / 2;
  }
}
void calculatePID()
{
  float avgPos = 0;
  int count = 0;
  for (int i = 0; i < 12; i++)
  {
    if (readMux(i) > sensorThresholds[i])
    {
      avgPos += (i - 5.5);
      count++;
    }
  }
  if (count > 0)
  {
    error = avgPos / count;
    integral = constrain(integral + error, -100, 100);
    float derivative = error - lastError;
    int adjustment = (int)(error * Kp + integral * Ki + derivative * Kd);
    lastError = error;
    driveMotors(constrain(BASE_SPEED + adjustment, 0, MAX_SPEED),

                constrain(BASE_SPEED - adjustment, 0, MAX_SPEED));
  }
  else
  {
    driveMotors(0, 0);
  }
}
void driveMotors(int left, int right)
{
  digitalWrite(AIN1, left >= 0);
  digitalWrite(AIN2, left < 0);
  analogWrite(PWMA, abs(left));
  digitalWrite(BIN1, right >= 0);
  digitalWrite(BIN2, right < 0);
  analogWrite(PWMB, abs(right));
}

int readMux(int channel)
{
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  delayMicroseconds(20);
  return analogRead(MUX_SIG);
}

void checkButton()
{
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    if (millis() - lastDebounceTime > debounceDelay)
    {
      motorRunning = !motorRunning;
      lastDebounceTime = millis();
    }
  }
}