// --- Pin Definitions ---
const int MUX_SIG = 34;
const int S0 = 16;
const int S1 = 5;
const int S2 = 18;
const int S3 = 19;

const int NUM_SENSORS = 12;

// --- Threshold Setting ---
// Change this value based on your raw solid black vs solid white readings.
// Anything ABOVE this number will trigger a '██' (Black Line)
const int THRESHOLD = 600; 

void setup() {
  Serial.begin(115200);

  // Set Multiplexer Control Pins as Output
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set Signal Pin as Input
  pinMode(MUX_SIG, INPUT);

  // ESP32 ADC Settings
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("Starting Visualizer & Raw Data Monitor...");
  Serial.println("Move the 15mm line under the sensors.");
  delay(1000); 
}

void loop() {
  String visualLine = "[";
  String numericLine = ""; // New string to hold the raw numbers
  
  // Read all 12 sensors and build both strings
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = readMux(i);
    
    // 1. Build the visual string (using 2 chars for both to prevent warping)
    if (value > THRESHOLD) {
      visualLine += "█"; 
    } 
    else {
      visualLine += " "; 
    }
    
    // 2. Build the numeric string (using \t for clean tabbed columns)
    numericLine += String(value) + "\t";
  }
  
  visualLine += "]";
  
  // Print the visual string, add a large gap, then print the numeric string
  Serial.print(visualLine);
  Serial.print("      |      "); // The gap
  Serial.println(numericLine);   // Print numbers and move to the next line

  // Delay 20ms (50 updates per second)
  delay(20); 
}

int readMux(int channel) {
  // Select the channel
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  
  // Settling time
  delayMicroseconds(80);
  
  // Read and return the voltage
  return analogRead(MUX_SIG);
}