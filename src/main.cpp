#include <Arduino.h>
#include <TMCStepper.h>

// ============================================
// CONFIGURATION DES PINS (D-Série selon schéma)
// ============================================

// MOTEUR 1 (U2)
#define EN1_PIN           0   // D0 (GPIO1)
#define STEP1_PIN         3   // D3 (GPIO4)
#define DIR1_PIN          4   // D4 (GPIO5)
#define TMC1_RX           14  // RX_1 (GPIO41)
#define TMC1_TX           15  // TX_1 (GPIO42)

// MOTEUR 2 (U3)
#define EN2_PIN           5   // D5 (GPIO6)
#define STEP2_PIN         10  // D10 (GPIO9)
#define DIR2_PIN          9   // D9 (GPIO8)
#define TMC2_RX           11  // RX_2 (GPIO11)
#define TMC2_TX           12  // TX_2 (GPIO12)

#define R_SENSE           0.11f
#define MICROSTEPS        16
#define RPM               30
#define MOTOR_STEPS       200

// ============================================
// OBJETS MOTEURS
// ============================================

// Driver 1 sur Serial1 (RX1/TX1)
TMC2209Stepper driver1(&Serial1, R_SENSE, 0x00);
// Driver 2 sur Serial2 (RX2/TX2)
TMC2209Stepper driver2(&Serial2, R_SENSE, 0x00);

// PROTOTYPES
void setupMotors();
void moveForward(int duration_ms);
void diagMotor(int motorNum, TMC2209Stepper &driver);

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
  // Activation des drivers (Low = Enabled)
  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, LOW);

  // Démarrage des deux bus UART distincts
  Serial1.begin(115200, SERIAL_8N1, TMC1_RX, TMC1_TX);
  Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);
  
  delay(500);
  setupMotors();

  Serial.println("Configuration terminée !");
}

void loop() {
  Serial.println("Mouvement en cours...");
  moveForward(3000);
  delay(1000);
}

// ============================================
// FONCTIONS
// ============================================

void setupMotors() {
  // Config Moteur 1
  driver1.begin();
  driver1.rms_current(1000);
  driver1.microsteps(MICROSTEPS);
  driver1.pwm_autoscale(true);
  
  // Config Moteur 2
  driver2.begin();
  driver2.rms_current(1000);
  driver2.microsteps(MICROSTEPS);
  driver2.pwm_autoscale(true);
  
  delay(100);
  diagMotor(1, driver1);
  diagMotor(2, driver2);
}

void moveForward(int duration_ms) {
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  
  float step_freq = (RPM * MOTOR_STEPS * MICROSTEPS) / 60.0f;
  int delay_us = (int)(1000000 / step_freq);

  unsigned long start_time = millis();
  while (millis() - start_time < (unsigned long)duration_ms) {
    digitalWrite(STEP1_PIN, HIGH);
    digitalWrite(STEP2_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
    delayMicroseconds(delay_us - 10);
  }
}

void diagMotor(int motorNum, TMC2209Stepper &driver) {
  uint32_t version = driver.version();
  if (version == 0 || version == 0xFFFFFFFF) {
    Serial.printf("Moteur %d UART : ÉCHEC\n", motorNum);
  } else {
    Serial.printf("Moteur %d UART OK. Version: 0x%X\n", motorNum, version);
  }
}