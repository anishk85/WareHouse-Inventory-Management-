/*
 * ESP32 Stepper Motor Controller for Mecanum Robot
 * Controls 2 NEMA23 stepper motors via drivers (one per wheel)
 * Communication: Serial @ 115200 baud
 * Protocol: "V,steps_per_sec_motor1,steps_per_sec_motor2\n"
 * 
 * Hardware Setup:
 * - ESP1 (Front): Controls Front Left (FL) + Front Right (FR) wheels
 * - ESP2 (Back):  Controls Back Left (BL) + Back Right (BR) wheels
 * 
 * Each motor connects to a separate stepper driver
 */

// ========== CONFIGURATION - MODIFY FOR YOUR WIRING ==========

// Set ESP ID: 1 for FRONT ESP, 2 for BACK ESP
const int ESP_ID = 1;  // ⚠️ CHANGE TO 2 FOR BACK ESP

// Motor 1 pins (Front Left or Back Left wheel)
const int MOTOR1_STEP_PIN = 12;
const int MOTOR1_DIR_PIN = 14;
const int MOTOR1_ENABLE_PIN = 27;

// Motor 2 pins (Front Right or Back Right wheel)
const int MOTOR2_STEP_PIN = 26;
const int MOTOR2_DIR_PIN = 25;
const int MOTOR2_ENABLE_PIN = 33;

// Stepper driver configuration
const int MICROSTEPS = 1;  // Set according to your driver DIP switches (1, 2, 4, 8, 16, 32)
const int STEPS_PER_REV = 200 * MICROSTEPS;  // NEMA23 = 200 steps/rev (1.8°)

// Motor control parameters
const float MAX_ACCEL = 2000.0;  // steps/sec² (adjust for smooth acceleration)
const float MAX_SPEED = 3000.0;  // steps/sec (safety limit)

// ================================================================

// Motor state structure
struct Motor {
  int step_pin;
  int dir_pin;
  int enable_pin;
  float target_steps_per_sec;
  float current_steps_per_sec;
  unsigned long last_step_time;
  unsigned long step_interval_us;
  bool direction;  // true = forward, false = backward
  const char* name;
};

Motor motor1, motor2;

// Control loop timing
const float CONTROL_FREQ = 100.0;  // Hz
const unsigned long CONTROL_PERIOD_US = 1000000.0 / CONTROL_FREQ;
unsigned long last_control_time = 0;

// Serial communication
String inputString = "";
boolean stringComplete = false;

// Watchdog timer (stops motors if no command received)
unsigned long last_command_time = 0;
const unsigned long WATCHDOG_TIMEOUT = 500;  // ms

// Statistics
unsigned long loop_count = 0;
unsigned long last_stats_time = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  // Initialize Motor 1
  motor1.step_pin = MOTOR1_STEP_PIN;
  motor1.dir_pin = MOTOR1_DIR_PIN;
  motor1.enable_pin = MOTOR1_ENABLE_PIN;
  motor1.target_steps_per_sec = 0.0;
  motor1.current_steps_per_sec = 0.0;
  motor1.last_step_time = 0;
  motor1.step_interval_us = 0;
  motor1.direction = true;
  motor1.name = (ESP_ID == 1) ? "FL" : "BL";
  
  pinMode(motor1.step_pin, OUTPUT);
  pinMode(motor1.dir_pin, OUTPUT);
  pinMode(motor1.enable_pin, OUTPUT);
  digitalWrite(motor1.enable_pin, LOW);  // Enable motor (active LOW)
  digitalWrite(motor1.step_pin, LOW);
  digitalWrite(motor1.dir_pin, LOW);
  
  // Initialize Motor 2
  motor2.step_pin = MOTOR2_STEP_PIN;
  motor2.dir_pin = MOTOR2_DIR_PIN;
  motor2.enable_pin = MOTOR2_ENABLE_PIN;
  motor2.target_steps_per_sec = 0.0;
  motor2.current_steps_per_sec = 0.0;
  motor2.last_step_time = 0;
  motor2.step_interval_us = 0;
  motor2.direction = true;
  motor2.name = (ESP_ID == 1) ? "FR" : "BR";
  
  pinMode(motor2.step_pin, OUTPUT);
  pinMode(motor2.dir_pin, OUTPUT);
  pinMode(motor2.enable_pin, OUTPUT);
  digitalWrite(motor2.enable_pin, LOW);  // Enable motor (active LOW)
  digitalWrite(motor2.step_pin, LOW);
  digitalWrite(motor2.dir_pin, LOW);
  
  inputString.reserve(100);
  
  // Startup message
  Serial.println("\n========================================");
  Serial.println("ESP32 Mecanum Stepper Controller");
  Serial.print("ESP_ID: ");
  Serial.println(ESP_ID);
  Serial.print("Position: ");
  Serial.println((ESP_ID == 1) ? "FRONT (FL + FR)" : "BACK (BL + BR)");
  Serial.print("Motor 1 (");
  Serial.print(motor1.name);
  Serial.print("): STEP=");
  Serial.print(MOTOR1_STEP_PIN);
  Serial.print(" DIR=");
  Serial.print(MOTOR1_DIR_PIN);
  Serial.print(" EN=");
  Serial.println(MOTOR1_ENABLE_PIN);
  Serial.print("Motor 2 (");
  Serial.print(motor2.name);
  Serial.print("): STEP=");
  Serial.print(MOTOR2_STEP_PIN);
  Serial.print(" DIR=");
  Serial.print(MOTOR2_DIR_PIN);
  Serial.print(" EN=");
  Serial.println(MOTOR2_ENABLE_PIN);
  Serial.print("Microsteps: ");
  Serial.println(MICROSTEPS);
  Serial.print("Steps/Rev: ");
  Serial.println(STEPS_PER_REV);
  Serial.println("Protocol: V,steps_per_sec_m1,steps_per_sec_m2");
  Serial.println("Ready!");
  Serial.println("========================================\n");
  
  last_command_time = millis();
  last_stats_time = millis();
}

void loop() {
  unsigned long current_time = micros();
  loop_count++;
  
  // Read serial commands
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  
  // Process command
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
    last_command_time = millis();
  }
  
  // Watchdog: stop motors if no command received
  if (millis() - last_command_time > WATCHDOG_TIMEOUT) {
    if (motor1.target_steps_per_sec != 0.0 || motor2.target_steps_per_sec != 0.0) {
      motor1.target_steps_per_sec = 0.0;
      motor2.target_steps_per_sec = 0.0;
      Serial.println("WATCHDOG: Motors stopped (no command)");
    }
  }
  
  // Control loop - update velocities with acceleration limiting
  if (current_time - last_control_time >= CONTROL_PERIOD_US) {
    updateMotorVelocity(motor1);
    updateMotorVelocity(motor2);
    last_control_time = current_time;
  }
  
  // Step motors
  stepMotor(motor1, current_time);
  stepMotor(motor2, current_time);
  
  // Print statistics every 5 seconds
  if (millis() - last_stats_time > 5000) {
    printStats();
    last_stats_time = millis();
  }
}

void parseCommand(String cmd) {
  // Expected format: "V,steps_per_sec_m1,steps_per_sec_m2"
  if (cmd.startsWith("V,")) {
    cmd = cmd.substring(2);  // Remove "V,"
    
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      String m1_str = cmd.substring(0, commaIndex);
      String m2_str = cmd.substring(commaIndex + 1);
      
      float m1_speed = m1_str.toFloat();
      float m2_speed = m2_str.toFloat();
      
      // Apply speed limits
      motor1.target_steps_per_sec = constrain(m1_speed, -MAX_SPEED, MAX_SPEED);
      motor2.target_steps_per_sec = constrain(m2_speed, -MAX_SPEED, MAX_SPEED);
      
      // Optional: Echo command for debugging
      // Serial.print("CMD: ");
      // Serial.print(motor1.name);
      // Serial.print("=");
      // Serial.print(motor1.target_steps_per_sec);
      // Serial.print(" ");
      // Serial.print(motor2.name);
      // Serial.print("=");
      // Serial.println(motor2.target_steps_per_sec);
    }
  } else if (cmd.startsWith("STATUS")) {
    printStatus();
  } else if (cmd.startsWith("STOP")) {
    motor1.target_steps_per_sec = 0.0;
    motor2.target_steps_per_sec = 0.0;
    Serial.println("EMERGENCY STOP");
  }
}

void updateMotorVelocity(Motor &motor) {
  float error = motor.target_steps_per_sec - motor.current_steps_per_sec;
  float max_change = MAX_ACCEL / CONTROL_FREQ;
  
  // Apply acceleration limiting
  if (abs(error) > max_change) {
    motor.current_steps_per_sec += (error > 0) ? max_change : -max_change;
  } else {
    motor.current_steps_per_sec = motor.target_steps_per_sec;
  }
  
  // Update direction pin
  if (motor.current_steps_per_sec >= 0) {
    motor.direction = true;
    digitalWrite(motor.dir_pin, HIGH);
  } else {
    motor.direction = false;
    digitalWrite(motor.dir_pin, LOW);
  }
  
  // Calculate step interval (microseconds between steps)
  float abs_speed = abs(motor.current_steps_per_sec);
  if (abs_speed > 0.1) {
    motor.step_interval_us = 1000000.0 / abs_speed;
  } else {
    motor.step_interval_us = 0;  // Motor stopped
  }
}

void stepMotor(Motor &motor, unsigned long current_time) {
  if (motor.step_interval_us == 0) {
    return;  // Motor stopped
  }
  
  if (current_time - motor.last_step_time >= motor.step_interval_us) {
    // Generate step pulse
    digitalWrite(motor.step_pin, HIGH);
    delayMicroseconds(2);  // Minimum pulse width (check your driver datasheet)
    digitalWrite(motor.step_pin, LOW);
    
    motor.last_step_time = current_time;
  }
}

void printStatus() {
  Serial.println("\n--- STATUS ---");
  Serial.print("ESP_ID: ");
  Serial.println(ESP_ID);
  Serial.print(motor1.name);
  Serial.print(": target=");
  Serial.print(motor1.target_steps_per_sec);
  Serial.print(" current=");
  Serial.print(motor1.current_steps_per_sec);
  Serial.print(" dir=");
  Serial.println(motor1.direction ? "FWD" : "REV");
  Serial.print(motor2.name);
  Serial.print(": target=");
  Serial.print(motor2.target_steps_per_sec);
  Serial.print(" current=");
  Serial.print(motor2.current_steps_per_sec);
  Serial.print(" dir=");
  Serial.println(motor2.direction ? "FWD" : "REV");
  Serial.println("-------------\n");
}

void printStats() {
  Serial.print("STATS: ESP");
  Serial.print(ESP_ID);
  Serial.print(" | ");
  Serial.print(motor1.name);
  Serial.print("=");
  Serial.print(motor1.current_steps_per_sec, 1);
  Serial.print(" ");
  Serial.print(motor2.name);
  Serial.print("=");
  Serial.print(motor2.current_steps_per_sec, 1);
  Serial.print(" | Loops/sec=");
  Serial.println(loop_count / 5);
  loop_count = 0;
}