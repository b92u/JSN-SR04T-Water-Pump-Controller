#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AsyncDelay.h>
#include <jsnsr04t.h>
#include <NeoSWSerial.h>

// OLED I2C display definitions
#define SCREEN_WIDTH 128 // Width in pixels
#define SCREEN_HEIGHT 64 // Height in pixels

// Bluetooth configuration
NeoSWSerial BTSerial(2, 3); // RX, TX

// I2C display address
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // I2C address for the display

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Distance sensor pin definitions
#define ECHO_PIN 11
#define TRIGGER_PIN 12
#define SIGNAL 8

// Ultrasonic sensor initialization
JsnSr04T ultrasonicSensor(ECHO_PIN, TRIGGER_PIN, LOG_LEVEL_VERBOSE);
AsyncDelay measureDelay;

// Global variables
const int threshold = 30;
const int variance = 2;
unsigned long activationDuration = 5000;
unsigned long lastActivationTime = 0;
unsigned long lastResetTime = 0;
bool pumpActive = false;
bool waitingAfterActivation = false; // Variable to track the waiting state
const int bufferSize = 5; // Number of readings for average
int distanceBuffer[bufferSize];
int bufferIndex = 0;
int activationCount = 0;
int lastValidDistance = -1; // Last valid distance for comparison

void setup() {
  Serial.begin(115200);
  ultrasonicSensor.begin(Serial);

  BTSerial.begin(9600);
  Serial.println(F("Bluetooth initialized"));

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  Serial.println(F("SSD1306 initialized successfully"));
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  pinMode(SIGNAL, OUTPUT);

  measureDelay.start(1000, AsyncDelay::MILLIS);

  // Record the initial time for the 24-hour reset
  lastResetTime = millis();

  resetBuffer();
}

void loop() {
  // Check if 24 hours have passed since the last reset
  if (millis() - lastResetTime >= 86400000) {
    activationCount = 0;
    lastResetTime = millis();
  }

  if (measureDelay.isExpired()) {
    int distance = ultrasonicSensor.readDistance();
    if (distance < 0) {
      Serial.println(F("Sensor read error."));
    } else {
      Serial.print(F("Measured distance: "));
      Serial.print(distance);
      Serial.println(F(" cm"));

      // Update the distance buffer if the reading is valid
      if (lastValidDistance == -1 || abs(distance - lastValidDistance) <= variance) {
        lastValidDistance = distance;
        distanceBuffer[bufferIndex] = distance;
        bufferIndex = (bufferIndex + 1) % bufferSize;
      }

      // Calculate the average of the buffer
      int sum = 0;
      int validCount = 0;
      for (int i = 0; i < bufferSize; i++) {
        if (distanceBuffer[i] >= 0) {
          sum += distanceBuffer[i];
          validCount++;
        }
      }
      int avgDistance = (validCount > 0) ? sum / validCount : -1;

      Serial.print(F("Average distance: "));
      Serial.println(avgDistance);

      display.clearDisplay();
      display.setCursor(0, 20);
      display.print(F("Distance: "));
      if (avgDistance >= 0) {
        display.print(avgDistance);
      } else {
        display.print("Error");
      }
      display.print(F(" cm"));
      display.setCursor(0, 40);
      display.print(F("Activations: "));
      display.print(activationCount);
      display.display();

      if (!waitingAfterActivation && !pumpActive && avgDistance > 0 && avgDistance <= threshold) {
        activatePump();
      }

      if (pumpActive && (millis() - lastActivationTime >= activationDuration)) {
        digitalWrite(SIGNAL, LOW);
        pumpActive = false;
        Serial.println(F("Pump deactivated."));
      }

      if (waitingAfterActivation && (millis() - lastActivationTime >= activationDuration + 5000)) {
        waitingAfterActivation = false; // Allow reactivation after 5 seconds of waiting
      }
    }

    measureDelay.repeat();
  }

  // Check for commands via Bluetooth
  if (BTSerial.available()) {
    delay(100);
    char command[20];
    int i = 0;
    while (BTSerial.available() && i < sizeof(command) - 1) {
      char c = BTSerial.read();
      if (c == '\n') break;
      if (isPrintable(c)) { // Check if character is printable ASCII
        command[i++] = c;
      }
    }
    command[i] = '\0'; // Ensure null-terminated string

    Serial.print(F("Received raw command: "));
    Serial.println(command);

    // Verify each character in the command to debug potential issues
    Serial.print(F("Command characters: "));
    for (int j = 0; j < i; j++) {
      Serial.print(command[j], HEX);
      Serial.print(" ");
    }
    Serial.println();

    if (strncmp(command, "DURATION", 8) == 0) {
      int durationValue = atoi(command + 8);
      if (durationValue >= 5 && durationValue <= 20) {
        activationDuration = durationValue * 1000;
        BTSerial.print(F("New duration: "));
        BTSerial.print(activationDuration / 1000);
        BTSerial.println(F(" seconds"));
        Serial.print(F("New duration set: "));
        Serial.print(activationDuration / 1000);
        Serial.println(F(" seconds"));

        display.clearDisplay();
        display.setCursor(0, 20);
        display.print(F("New duration: "));
        display.print(activationDuration / 1000);
        display.print(F(" sec"));
        display.display();
        delay(1000);
      } else {
        BTSerial.println(F("Invalid duration value"));
        Serial.println(F("Invalid duration value"));
      }
    } else if (strncmp(command, "ACTIVATE", 8) == 0) {
      activatePump();
      Serial.println(F("Pump activated via Bluetooth."));
    } else {
      BTSerial.println(F("Unknown command"));
      Serial.println(F("Unknown command received"));
    }
  }
}

void activatePump() {
  digitalWrite(SIGNAL, HIGH);
  lastActivationTime = millis();
  pumpActive = true;
  waitingAfterActivation = true;
  activationCount++;
  Serial.println(F("Pump activated."));
}

void resetBuffer() {
  for (int i = 0; i < bufferSize; i++) {
    distanceBuffer[i] = -1;
  }
  lastValidDistance = -1; // Reset last valid distance
}
