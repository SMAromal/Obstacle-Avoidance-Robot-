# Obstacle-Avoidance-Robot-
This project showcases an obstacle avoidance robot using an ESP32 microcontroller andes source code ,its accessible for both beginners and experi various sensors, including ultrasonic, infrared, to autonomously navigate its environment and avoid obstacles in real-time.

#include <MQ135.h>

#define MIC_PIN_1 A0
#define MIC_PIN_2 A1
#define MIC_PIN_3 A2
#define MIC_PIN_4 A3

#define MOTION_SENSOR_PIN_1 2
#define MOTION_SENSOR_PIN_2 3
#define MOTION_SENSOR_PIN_3 4

#define ULTRASONIC_TRIGGER_PIN_1 5
#define ULTRASONIC_ECHO_PIN_1 6

#define ULTRASONIC_TRIGGER_PIN_2 7
#define ULTRASONIC_ECHO_PIN_2 8

#define ULTRASONIC_TRIGGER_PIN_3 9
#define ULTRASONIC_ECHO_PIN_3 10

#define MOTOR_PIN_1 11   // Motor 1 control pin 1
#define MOTOR_PIN_2 12   // Motor 1 control pin 2
#define MOTOR_PIN_3 13   // Motor 2 control pin 1
#define MOTOR_PIN_4 14   // Motor 2 control pin 2

#define MQ135_PIN A4     // Analog pin where MQ135 sensor is connected
#define LED_PIN 15       // GPIO pin for status LED

MQ135 mq135(MQ135_PIN);

void setup() {
  Serial.begin(9600);
  
  // Set pin modes
  pinMode(MIC_PIN_1, INPUT);
  pinMode(MIC_PIN_2, INPUT);
  pinMode(MIC_PIN_3, INPUT);
  pinMode(MIC_PIN_4, INPUT);
  
  pinMode(MOTION_SENSOR_PIN_1, INPUT);
  pinMode(MOTION_SENSOR_PIN_2, INPUT);
  pinMode(MOTION_SENSOR_PIN_3, INPUT);
  
  pinMode(ULTRASONIC_TRIGGER_PIN_1, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN_1, INPUT);
  
  pinMode(ULTRASONIC_TRIGGER_PIN_2, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN_2, INPUT);
  
  pinMode(ULTRASONIC_TRIGGER_PIN_3, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN_3, INPUT);
  
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(MOTOR_PIN_3, OUTPUT);
  pinMode(MOTOR_PIN_4, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Check for sound detection
  int soundDirection = detectSoundDirection();
  if (soundDirection != -1) {
    soundDetectedDirection = soundDirection;
    soundDetectedTime = millis();
  }
  
  // Move towards the direction where sound was detected and wait
  if (soundDetectedDirection != -1 && millis() - soundDetectedTime < 10000) {
    moveTowardsDirection(soundDetectedDirection);
    delay(1000); // Wait for 10 seconds
  } else {
    // Scan surroundings for motion and ultrasonic distance
    bool motionDetected = scanMotion();
    bool obstacleDetected = scanObstacle();
    bool gasDetected = detectGas();

    // If motion, obstacle, or gas detected, stop and wait for the next sound
    if (motionDetected || obstacleDetected || gasDetected) {
      stopMotors();
      delay(5000); // Wait for 5 seconds before scanning for sound again
      soundDetectedDirection = -1; // Reset sound detection direction
    }
  }
}

// Function to detect sound direction
int detectSoundDirection() {
  // Read microphone sensor values
  int sound1 = analogRead(MIC_PIN_1);
  int sound2 = analogRead(MIC_PIN_2);
  int sound3 = analogRead(MIC_PIN_3);
  int sound4 = analogRead(MIC_PIN_4);
  
  // Assuming each microphone corresponds to a specific direction
  if (sound1 > 500) return 0; // Front
  if (sound2 > 500) return 1; // Back
  if (sound3 > 500) return 2; // Left
  if (sound4 > 500) return 3; // Right
  return -1;
}

// Function to move towards a specific direction
void moveTowardsDirection(int direction) {
  // Assuming direction 0 - Front, 1 - Back, 2 - Left, 3 - Right
  
  // Move motors accordingly
  switch (direction) {
    case 0: // Front
      // Move forward
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
      digitalWrite(MOTOR_PIN_3, HIGH);
      digitalWrite(MOTOR_PIN_4, LOW);
      break;
    case 1: // Back
      // Move backward
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
      digitalWrite(MOTOR_PIN_3, LOW);
      digitalWrite(MOTOR_PIN_4, HIGH);
      break;
    case 2
: // Left
      // Move left
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
      digitalWrite(MOTOR_PIN_3, HIGH);
      digitalWrite(MOTOR_PIN_4, LOW);
      break;
    case 3: // Right
      // Move right
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
      digitalWrite(MOTOR_PIN_3, LOW);
      digitalWrite(MOTOR_PIN_4, HIGH);
      break;
  }
}

// Function to scan for motion using PIR sensors
bool scanMotion() {
  bool motionDetected = false;
  if (digitalRead(MOTION_SENSOR_PIN_1) == HIGH ||
      digitalRead(MOTION_SENSOR_PIN_2) == HIGH ||
      digitalRead(MOTION_SENSOR_PIN_3) == HIGH) {
    Serial.println("Motion detected!");
    motionDetected = true;
  }
  return motionDetected;
}

// Function to scan for obstacles using ultrasonic sensors
bool scanObstacle() {
  bool obstacleDetected = false;
  // Assuming Ultrasonic sensor scanning implementation
  // If an obstacle is detected within a certain range, set obstacleDetected = true
  return obstacleDetected;
}

// Function to detect gas using MQ135 sensor
bool detectGas() {
  // Read gas value from MQ135 sensor
  float gasValue = mq135.getPPM();
  Serial.print("Gas value: ");
  Serial.println(gasValue);

  // Check if gas value exceeds a certain threshold
  if (gasValue > 50) {
    Serial.println("Gas detected!");
    digitalWrite(LED_PIN, HIGH); // Turn on LED as gas indicator
    return true;
  } else {
    digitalWrite(LED_PIN, LOW); // Turn off LED
    return false;
  }
}

// Function to stop motors
void stopMotors() {
  // Stop motors by setting all motor pins to LOW
  digitalWrite(MOTOR_PIN_1, LOW);
  digitalWrite(MOTOR_PIN_2, LOW);
  digitalWrite(MOTOR_PIN_3, LOW);
  digitalWrite(MOTOR_PIN_4, LOW);
}
