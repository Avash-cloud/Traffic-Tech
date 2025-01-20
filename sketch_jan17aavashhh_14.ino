#include <WiFi.h>
#include <ESP32Servo.h>

// Wi-Fi credentials
const char* ssid = "IIC_WIFI";
const char* password = "!tah@rIntl2025";

// Ultrasonic Sensor for Pedestrian Detection
#define TRIG_PIN 5
#define ECHO_PIN 18

// Traffic Light LEDs
#define RED_LED 23
#define GREEN_LED 4

// Servo Pin
#define SERVO_PIN 25

// Constants
#define DISTANCE_THRESHOLD 5    // Distance in cm to detect a pedestrian
#define PEDESTRIAN_CROSS_TIME 10000 // 10 seconds for pedestrian to cross

// Servo object
Servo servo;

void setup() {
  // Start Serial Monitor for debugging
  Serial.begin(115200);

  // Wi-Fi Connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);  // Wait for connection
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");

  // Pin Modes for Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Pin Modes for Traffic Light LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Initialize Servo
  servo.attach(SERVO_PIN);

  // Initial State: Green Light for Vehicles
  digitalWrite(RED_LED, LOW);   // Red light OFF
  digitalWrite(GREEN_LED, HIGH); // Green light ON

  // Set initial servo position (open)
  servo.write(0);  // 0 degrees (open)
}

void loop() {
  // Measure distance from the ultrasonic sensor
  int distance = measureDistance(TRIG_PIN, ECHO_PIN);
  
  // Print the distance to Serial Monitor for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check if a pedestrian is detected (within 5 cm range)
  if (distance >= 0 && distance <= DISTANCE_THRESHOLD) {
    // Pedestrian detected: Turn on Red Light and activate Servo
    digitalWrite(RED_LED, HIGH);  // Red light ON
    digitalWrite(GREEN_LED, LOW); // Green light OFF

    // Close the road with Servo (90 degrees)
    servo.write(90);  // 90 degrees (closed)
    Serial.println("Pedestrian detected! Activating Servo.");

    // Wait for pedestrian to cross (PEDESTRIAN_CROSS_TIME)
    delay(PEDESTRIAN_CROSS_TIME);

    // After delay, return to normal operation
    servo.write(0);  // Open the road (0 degrees)

    // Set traffic light back to Green for vehicles
    digitalWrite(RED_LED, LOW);   // Red light OFF
    digitalWrite(GREEN_LED, HIGH); // Green light ON

    Serial.println("Pedestrian crossing complete. Returning to normal operation.");
  } else {
    // No pedestrian detected: Normal traffic light operation
    digitalWrite(RED_LED, LOW);   // Red light OFF
    digitalWrite(GREEN_LED, HIGH); // Green light ON

    // Do not move the servo; keep it at the last position
    Serial.println("No pedestrian detected. Green light ON for vehicles.");
  }

  delay(500); // Short delay for sensor stability
}

// Function to measure distance using ultrasonic sensor
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);  // Small delay to avoid incorrect readings
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Pulse duration (10 µs)
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);  // Measure pulse duration
  int distance = duration * 0.034 / 2;    // Convert to distance in cm

  return distance;
}
