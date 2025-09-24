#include <Servo.h>

// --- Pin Definitions ---
// X-Axis (ST1)
#define X_PUL 4
#define X_DIR 5
#define X_ENA 6
#define X_LIM1 7
#define X_LIM2 8

// Z-Axis (ST2)
#define Z_PUL 2
#define Z_DIR 3
#define Z_ENA 12
#define Z_LIM3 9
#define Z_LIM4 10

// Servo
#define SERVO_PIN 11

// --- State Machine ---
enum State {
  MOVE_X_FORWARD,
  MOVE_Z_DOWN_PICK,
  SERVO_PICK,
  MOVE_Z_UP_AFTER_PICK,
  MOVE_X_BACKWARD,
  MOVE_Z_DOWN_PLACE,
  SERVO_PLACE,
  MOVE_Z_UP_AFTER_PLACE
};

State currentState = MOVE_X_FORWARD;

// --- Global Variables ---
Servo myServo;

// --- Function Prototypes ---
void makeStep(int pin, int delayMicro);
void moveUntilLimit(int pulPin, int dirPin, int limitPin);

void setup() {
  // --- Motor Pins ---
  pinMode(X_PUL, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_ENA, OUTPUT);
  pinMode(Z_PUL, OUTPUT);
  pinMode(Z_DIR, OUTPUT);
  pinMode(Z_ENA, OUTPUT);

  // --- Limit Switches ---
  // Limit switches are active LOW (connected to GND when pressed)
  pinMode(X_LIM1, INPUT_PULLUP);
  pinMode(X_LIM2, INPUT_PULLUP);
  pinMode(Z_LIM3, INPUT_PULLUP);
  pinMode(Z_LIM4, INPUT_PULLUP);

  // --- Initial Motor State ---
  // Enable both stepper motors (active LOW)
  digitalWrite(X_ENA, LOW);
  digitalWrite(Z_ENA, LOW);

  // --- Servo Setup ---
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Initial position (open gripper)
  delay(500); // Allow servo to reach initial position
  
  // --- Serial Communication for Debugging ---
  Serial.begin(9600);
  Serial.println("Arduino Pick and Place Initialized");
  Serial.println("Starting cycle...");
}

void loop() {
  switch (currentState) {
    case MOVE_X_FORWARD:
      Serial.println("State: Moving X forward to pick location.");
      digitalWrite(X_DIR, HIGH); // Set direction
      moveUntilLimit(X_PUL, X_DIR, X_LIM2); // Move until limit switch is hit
      currentState = MOVE_Z_DOWN_PICK;
      delay(300); // Small delay to debounce
      break;

    case MOVE_Z_DOWN_PICK:
      Serial.println("State: Moving Z down to pick object.");
      digitalWrite(Z_DIR, LOW); // Set direction
      moveUntilLimit(Z_PUL, Z_DIR, Z_LIM3); // Move until limit switch is hit
      currentState = SERVO_PICK;
      delay(300);
      break;

    case SERVO_PICK:
      Serial.println("State: Closing servo to pick up object.");
      // Move servo to grab position
      for (int pos = 0; pos <= 120; pos += 5) {
        myServo.write(pos);
        delay(15); // Slightly longer delay for servo movement
      }
      delay(1000); // Wait for a moment to secure the object
      currentState = MOVE_Z_UP_AFTER_PICK;
      break;

    case MOVE_Z_UP_AFTER_PICK:
      Serial.println("State: Moving Z up with object.");
      digitalWrite(Z_DIR, HIGH); // Set direction
      moveUntilLimit(Z_PUL, Z_DIR, Z_LIM4); // Move until limit switch is hit
      currentState = MOVE_X_BACKWARD;
      delay(300);
      break;

    case MOVE_X_BACKWARD:
      Serial.println("State: Moving X backward to drop location.");
      digitalWrite(X_DIR, LOW); // Set direction
      moveUntilLimit(X_PUL, X_DIR, X_LIM1); // Move until limit switch is hit
      currentState = MOVE_Z_DOWN_PLACE;
      delay(300);
      break;

    case MOVE_Z_DOWN_PLACE:
      Serial.println("State: Moving Z down to place object.");
      digitalWrite(Z_DIR, LOW); // Set direction
      moveUntilLimit(Z_PUL, Z_DIR, Z_LIM3); // Move until limit switch is hit
      currentState = SERVO_PLACE;
      delay(300);
      break;

    case SERVO_PLACE:
      Serial.println("State: Opening servo to place object.");
      // Move servo to release position
      for (int pos = 120; pos >= 0; pos -= 5) {
        myServo.write(pos);
        delay(15); // Slightly longer delay for servo movement
      }
      delay(1000); // Wait for a moment to ensure release
      currentState = MOVE_Z_UP_AFTER_PLACE;
      break;

    case MOVE_Z_UP_AFTER_PLACE:
      Serial.println("State: Moving Z up after place, completing cycle.");
      digitalWrite(Z_DIR, HIGH); // Set direction
      moveUntilLimit(Z_PUL, Z_DIR, Z_LIM4); // Move until limit switch is hit
      currentState = MOVE_X_FORWARD; // Restart cycle
      delay(300);
      Serial.println("Cycle complete. Starting new cycle...");
      break;
  }
}

// --- Helper Functions ---

/**
 * @brief Makes a single step for the stepper motor.
 * @param pin The PULSE pin of the stepper motor driver.
 * @param delayMicro The delay in microseconds for each step.
 */
void makeStep(int pin, int delayMicro) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(delayMicro);
  digitalWrite(pin, LOW);
  delayMicroseconds(delayMicro);
}

/**
 * @brief Moves the motor until a limit switch is triggered.
 * @param pulPin The PULSE pin of the stepper motor.
 * @param dirPin The DIRECTION pin of the stepper motor.
 * @param limitPin The limit switch pin to monitor.
 */
void moveUntilLimit(int pulPin, int dirPin, int limitPin) {
  // Wait for the limit switch to be pressed (active LOW)
  while (digitalRead(limitPin) == HIGH) {
    makeStep(pulPin, 800); // Use a fixed speed for this function
    // Add a small delay to prevent watchdog timer issues
    if (millis() % 100 == 0) {
      delayMicroseconds(10);
    }
  }
}