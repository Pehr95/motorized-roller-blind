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
  if (ina219.getCurrent_mA() > 200) {
    motorStalled = true;
    Serial.print("Bus Voltage:   "); Serial.print(ina219.getBusVoltage_V()); Serial.println(" V");
    Serial.print("Current mA:    "); Serial.print(ina219.getCurrent_mA()); Serial.println(" mA");
  }

  if(!motorStalled) {
    if (digitalRead(UP_BUTTON_PIN) == HIGH) {
    setMotor(255, UP);
    Serial.println("UP");
    }
    if (digitalRead(DOWN_BUTTON_PIN) == HIGH) {
      setMotor(255, DOWN);
      Serial.println("DOWN");
    }

    if (digitalRead(UP_BUTTON_PIN) == LOW && digitalRead(DOWN_BUTTON_PIN) == LOW) {
      setMotor(0, COAST);
    }
  }
  else {
    setMotor(0, COAST);
  }

  Serial.print("Bus Voltage:   "); Serial.print(ina219.getBusVoltage_V()); Serial.println(" V");
  Serial.print("Current mA:    "); Serial.print(ina219.getCurrent_mA()); Serial.println(" mA");
  delay(100);

}

