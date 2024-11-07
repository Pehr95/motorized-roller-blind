#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// Button pins
#define UP_BUTTON_PIN 2
#define DOWN_BUTTON_PIN 5

// Motor pins
#define MOTOR_IN1_PIN 4
#define MOTOR_IN2_PIN 3
#define MOTOR_PWM_CHANNEL1 0
#define MOTOR_PWM_CHANNEL2 1

// INA219 pins
#define INA219_SDA_PIN 6
#define INA219_SCL_PIN 7

// Encoder pins
#define ENCODER_C1_PIN 8
#define ENCODER_C2_PIN 1

volatile int encoderPosition = 0;
int lastEncoded = 0;

// Timers
unsigned long currentMillis = 0;
unsigned long stallStartTime = 0;  // Track when high current was first detected
unsigned long cooldownStartTime = 0;      // Tracks cooldown start time
unsigned long lastPrintTime = 0;  // Last time the current was printed


bool motorStalled = false;
Adafruit_INA219 ina219;

enum MotorDirection {
  UP,
  DOWN,
  COAST,
  BREAK,
};

void setMotor(byte speed, MotorDirection direction) {
  if (speed < 0) {
    speed = 0;
  }
  if (speed > 255) {
    speed = 255;
  }
  switch (direction) {
    case UP:
      ledcWrite(MOTOR_PWM_CHANNEL1, speed);
      ledcWrite(MOTOR_PWM_CHANNEL2, 0);
      break;
    case DOWN:
      ledcWrite(MOTOR_PWM_CHANNEL2, speed);
      ledcWrite(MOTOR_PWM_CHANNEL1, 0);
      break;
    case COAST:
      ledcWrite(MOTOR_PWM_CHANNEL1, 0);
      ledcWrite(MOTOR_PWM_CHANNEL2, 0);
      break;
    case BREAK:
      ledcWrite(MOTOR_PWM_CHANNEL1, 255);
      ledcWrite(MOTOR_PWM_CHANNEL2, 255);
      break;
  }
}

void updateEncoder() {
  // Read the two encoder channels
  int MSB = digitalRead(ENCODER_C1_PIN);
  int LSB = digitalRead(ENCODER_C2_PIN);

  int encoded = (MSB << 1) | LSB;  // Convert the 2-bit value
  int sum = (lastEncoded << 2) | encoded;  // Combine previous and current state

  // Determine direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  lastEncoded = encoded;  // Store this value for next time
}


const float D_max = 40.0;          // Diameter with fabric in mm
const float D_min = 26.0;          // Diameter without fabric in mm
const int pulses_per_rotation = 5860;

float getDistanceDown(int pulses_counted) {
  float D_avg = (D_max + D_min) / 2.0;   // Average diameter in mm
  float C_avg = 3.14159 * D_avg;         // Average circumference in mm
  float distance_per_pulse = C_avg / pulses_per_rotation;

  return pulses_counted * distance_per_pulse;  // Distance in mm
}


void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  // Initialize pins
  pinMode(UP_BUTTON_PIN, INPUT);
  pinMode(DOWN_BUTTON_PIN, INPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  ledcSetup(MOTOR_PWM_CHANNEL1, 2000, 8);
  ledcSetup(MOTOR_PWM_CHANNEL2, 2000, 8);
  ledcAttachPin(MOTOR_IN1_PIN, MOTOR_PWM_CHANNEL1);
  ledcAttachPin(MOTOR_IN2_PIN, MOTOR_PWM_CHANNEL2);
  Wire.begin(INA219_SDA_PIN, INA219_SCL_PIN);

  pinMode(ENCODER_C1_PIN, INPUT);
  pinMode(ENCODER_C2_PIN, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C2_PIN), updateEncoder, CHANGE);

  delay(1000);
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("Found INA219. Setup done!");
}

void loop() {
  currentMillis = millis();
  float current_mA = ina219.getCurrent_mA();  // Read current in mA
  //float busVoltage = ina219.getBusVoltage_V();  // Read bus voltage in V
  //float shuntVoltage = ina219.getShuntVoltage_mV();  // Read shunt voltage in mV

  /*
  if (lastPrintTime == 0 || currentMillis - lastPrintTime >= 1000) {
    Serial.print("Bus Voltage:   "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(ina219.getPower_mW()); Serial.println(" mW");
    Serial.println();
    lastPrintTime = currentMillis;
  }
  */


  Serial.println(encoderPosition); // Print the current position (or speed)
  double positionInCm = encoderPosition / (481.2 * (-1)) ;  // Convert encoder ticks to cm
  Serial.println(positionInCm); // Print the current in mA)
  Serial.print("GPT distance: ");
  Serial.println(getDistanceDown(encoderPosition)); // Print the current in mA)
  delay(100);


  if (current_mA > 300) {
    if (stallStartTime == 0 && !motorStalled) {
      // Start timing if current exceeds 70 mA and the motor is not stalled
      stallStartTime = currentMillis;
      Serial.print("Motor current high, starting timer... ");
      Serial.println(stallStartTime);
    } else if (currentMillis - stallStartTime >= 20 && !motorStalled) {
      // If the current is above 70 mA for more than 1000 ms, set motor to COAST
      motorStalled = true;
      cooldownStartTime = currentMillis;  // Start cooldown timer
      Serial.print("Motor stalled, shutting off... ");
      Serial.println(currentMillis);
    }
  } else if (!motorStalled) {
    // Reset stall timer if current drops below 70 mA and the motor is not stalled
    stallStartTime = 0;
  }


  if (!motorStalled) {
    if (digitalRead(UP_BUTTON_PIN) == HIGH) {
      setMotor(255, UP);
    } else if (digitalRead(DOWN_BUTTON_PIN) == HIGH) {
      setMotor(255, DOWN);
    } else {
      setMotor(0, COAST);
    }
  } else {
    setMotor(0, COAST);
  }

  // Reset motor stall flag after 5000 ms of cooldown
  if (motorStalled && (currentMillis - cooldownStartTime >= 5000)) {
    motorStalled = false;
    Serial.println("Motor stall cleared, restarting...");
  }
}

