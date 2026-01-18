#include <Arduino.h>
#include <TMCStepper.h>
#include <ESP32Servo.h>

// ============================================
// CONFIGURATION DES PINS (D-Série selon schéma)
// ============================================

// MOTEUR 1 (U2)
#define EN1_PIN           D4
#define STEP1_PIN         D5
#define DIR1_PIN          D3
#define TMC1_RX           D0
#define TMC1_TX           D8

// MOTEUR 2 (U3)
#define EN2_PIN           D9
#define STEP2_PIN         D6
#define DIR2_PIN          D3
#define TMC2_RX           D1
#define TMC2_TX           D7

// SERVO
#define SERVO_PIN         D10   // D8 on XIAO ESP32S3 (adjust if needed)
#define MAG_DETECT_PIN    GPIO_NUM_13   // D7 on XIAO ESP32S3

// CAPTEUR ULTRASONIC HC-SR04
#define US_TRIG_PIN       GPIO_NUM_12   // Trigger pin (adjust as needed)
#define US_ECHO_PIN       GPIO_NUM_11   // Echo pin (adjust as needed)
#define COLOR_SWITCH_PIN  GPIO_NUM_42   // D15 on XIAO ESP32S3

#define US_MAX_DISTANCE   300           // Max distance in cm

#define R_SENSE           0.11f
#define MICROSTEPS        4
#define RPM               150
#define MOTOR_STEPS       200  // Pas par révolution du moteur (1.8° = 200 pas)

// ============================================
// OBJETS MOTEURS
// ============================================

// Driver 1 sur Serial1 (RX1/TX1)
TMC2209Stepper driver1(&Serial1, R_SENSE, 0b00);
// Driver 2 sur Serial2 (RX2/TX2)
TMC2209Stepper driver2(&Serial2, R_SENSE, 0b00);

// Servo object
Servo wheelServo;

// PROTOTYPES
void setupMotors();
void moveForward(int duration_ms);
void diagMotor(int motorNum, TMC2209Stepper &driver);
void servoSweepLR();
float getUltrasonicDistance();

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(9600); 
  delay(2000);
  Serial.println("Initialisation du système...");

  // Configuration des broches de contrôle
  pinMode(EN1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(EN2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(COLOR_SWITCH_PIN, INPUT_PULLUP);
  
  // Activation des drivers (Low = Enabled)
  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, LOW);

  // Démarrage des bus UART
  Serial1.begin(115200, SERIAL_8N1, TMC1_RX, TMC1_TX);
  Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);
  
  delay(500);
  setupMotors();

  pinMode(MAG_DETECT_PIN, INPUT);

  // Init Servo
  wheelServo.attach(SERVO_PIN);
  wheelServo.write(90); // center

  // Init Ultrasonic
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  Serial.println("Configuration terminée !");

  Serial.print("Couleur : ");
  Serial.println(digitalRead(COLOR_SWITCH_PIN) == HIGH ? "JAUNE" : "BLEU");
}

void loop() {
  float distance = getUltrasonicDistance();
  Serial.printf("Distance: %.1f cm\n", distance);
  
  Serial.println("Mouvement en cours...");
  moveForward(3000);
  delay(1000);
  Serial.print("Magnet : ");
  Serial.println(digitalRead(MAG_DETECT_PIN) == HIGH ? "Présent" : "Absent");
}

// ============================================
// FONCTIONS
// ============================================

void setupMotors() {
  // Config Moteur 1
  driver1.begin();
  driver1.rms_current(1500);       // tune as needed
  driver1.microsteps(MICROSTEPS);
  driver1.pwm_autoscale(true);
  driver1.pwm_autograd(true);
  driver1.intpol(true);
  driver1.en_spreadCycle(false);  // use stealthChop
  driver1.pdn_disable(true);       // keep UART active
  driver1.mstep_reg_select(true);  // allow microstep config via UART
  
  // Config Moteur 2
  driver2.begin();
  driver2.rms_current(1500);
  driver2.microsteps(MICROSTEPS);
  driver2.pwm_autoscale(true);
  driver2.pwm_autograd(true);
  driver2.intpol(true);
  driver2.en_spreadCycle(false);  // use stealthChop
  driver2.pdn_disable(true);       // keep UART active
  driver2.mstep_reg_select(true);  // allow microstep config via UART
  
  delay(100);
  diagMotor(1, driver1);
  diagMotor(2, driver2);
}

void moveForward(int duration_ms) {
  // Pre-move: quick left-right sweep
  servoSweepLR();

  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  
  float step_freq = (RPM * MOTOR_STEPS * MICROSTEPS) / 60.0f;
  int accel_time_ms = 500;  // Acceleration duration in milliseconds
  
  unsigned long start_time = millis();
  unsigned long step_start_time = micros();
  
  while (millis() - start_time < (unsigned long)duration_ms) {
    unsigned long elapsed_ms = millis() - start_time;
    
    // Calculate current frequency based on acceleration ramp
    float current_freq = step_freq;
    if (elapsed_ms < (unsigned long)accel_time_ms) {
      // Linear acceleration ramp
      current_freq = step_freq * (float)elapsed_ms / (float)accel_time_ms;
    }
    
    int delay_us = (int)(1000000 / current_freq);
    
    unsigned long step_elapsed = micros() - step_start_time;
    if (step_elapsed >= (unsigned long)delay_us) {
      digitalWrite(STEP1_PIN, HIGH);
      digitalWrite(STEP2_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(STEP1_PIN, LOW);
      digitalWrite(STEP2_PIN, LOW);
      step_start_time = micros();
    }
  }

  // Post-move: quick left-right sweep
  servoSweepLR();
}

void diagMotor(int motorNum, TMC2209Stepper &driver) {
  uint32_t version = driver.version();
  if (version == 0 || version == 0xFFFFFFFF) {
    Serial.printf("Moteur %d UART : ÉCHEC\n", motorNum);
    if (motorNum == 1) {
      digitalWrite(LED_BUILTIN, HIGH); // Indicate error on built-in LED
    }
  } else {
    Serial.printf("Moteur %d UART OK. Version: 0x%X\n", motorNum, version);
  }
}

// ============================================
// SERVO SWEEP LEFT-RIGHT
// ============================================
void servoSweepLR() {
  // Simple left-right and back to center
  wheelServo.write(0);
  delay(150);
  wheelServo.write(170);
  delay(150);
  wheelServo.write(90);
  delay(150);
}

// ============================================
// ULTRASONIC DISTANCE MEASUREMENT (HC-SR04)
// ============================================
float getUltrasonicDistance() {
  // Send a 10 µs pulse to trigger
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
//   
//   // Measure echo pulse duration
  long duration = pulseIn(US_ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m max)
//   
  // Convert to distance: speed of sound = 343 m/s = 0.0343 cm/µs
  // distance = (duration * 0.0343) / 2
  float distance = (duration * 0.0343) / 2.0;
  
  // Return 0 if no echo or distance exceeds max
  if (duration == 0) {
    return 0.0;
  }
  
  return distance;
}