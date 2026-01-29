// ================= CONFIGURATION =================
// Set this to 'false' when running the robot on the track!
// Set to 'true' only when testing/calibrating sensors.
const bool DEBUG_MODE = true; 

// --- Pin Definitions (ESP32) ---
const int MUX_SIG = 34; // ADC1_CH6
const int S0 = 16;
const int S1 = 5;
const int S2 = 18;
const int S3 = 19;

// --- Sensor Settings ---
const int NUM_SENSORS = 12;
int sensorValues[NUM_SENSORS]; // Array to store readings (0-4095)

void setup() {
  // Use a fast baud rate for debugging to minimize lag
  Serial.begin(115200);

  // Set Control Pins as Output
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set Signal Pin as Input
  pinMode(MUX_SIG, INPUT);

  // --- ESP32 ADC Optimization ---
  // 12-bit resolution (Values 0 - 4095)
  analogReadResolution(12);
  // 11dB attenuation gives full 0-3.3V range
  analogSetAttenuation(ADC_11db); 
}

void loop() {
  readSensors();

  if (DEBUG_MODE) {
    printSensorData();
  }

  // --- YOUR PID / MOTOR LOGIC GOES HERE ---
  // calculatePID();
  // driveMotors();
}

// Function to read all 12 sensors efficiently
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // 1. Select the Channel
    // We use bitwise operators for speed and direct boolean conversion
    digitalWrite(S0, (i & 0x01) ? HIGH : LOW);
    digitalWrite(S1, (i & 0x02) ? HIGH : LOW);
    digitalWrite(S2, (i & 0x04) ? HIGH : LOW);
    digitalWrite(S3, (i & 0x08) ? HIGH : LOW);

    // 2. Settling Time
    // A tiny delay is needed for the Mux internal capacitor to switch
    // 2 microseconds is usually sufficient for ESP32 + 4051 Mux
    delayMicroseconds(2); 

    // 3. Read and Store
    sensorValues[i] = analogRead(MUX_SIG);
  }
}

// Helper function to view data (Disabled during race)
void printSensorData() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t"); // Tab space
  }
  Serial.println();
  delay(50); // Small delay only when debugging to make text readable
}