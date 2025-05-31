#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;

// Pines motores
const int pinPWMA = 5;
const int pinAIN2 = 2;
const int pinAIN1 = 0;
const int pinPWMB = 17;
const int pinBIN1 = 4;
const int pinBIN2 = 16;
const int pinSTBY = 15;

// Variables del sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x = 0.0, ang_y = 0.0;
float ang_x_prev = 0.0, ang_y_prev = 0.0;

// PID
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float integral = 0.0;
float derivative = 0.0;
float previous_error = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  sensor.initialize();

  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

  tiempo_prev = millis();

}

void loop() {
  // Leer datos del MPU6050
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Calcular ángulos con acelerómetro
  float accel_ang_x = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);

  // Calcular tiempo
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // Filtro complementario
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_x_prev = ang_x;

  // PID
  float desired_angle = 0.0;
  float error = desired_angle - ang_x;

  integral += error * dt;
  derivative = (error - previous_error) / dt;

  float motor_speed = (Kp * error + Ki * integral + Kd * derivative);
  previous_error = error;

  // Control de motores
  enableMotors();
  moveMotor(pinPWMA, pinAIN1, pinAIN2, motor_speed);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, motor_speed);

  // Monitor serial
  Serial.print("Ángulo X: "); Serial.print(ang_x);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Salida PID: "); Serial.print(motor_speed);
  Serial.print(" | Kp: "); Serial.print(Kp);
  Serial.print(" | Ki: "); Serial.print(Ki);
  Serial.print(" | Kd: "); Serial.println(Kd);

  // Lectura de comandos desde Serial
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'q': Kp += 1; break;
      case 'a': Kp -= 1; break;
      case 'w': Ki += 0.01; break;
      case 's': Ki -= 0.01; break;
      case 'e': Kd += 0.1; break;
      case 'd': Kd -= 0.1; break;
    }
    // Imprimir nueva configuración
    Serial.print("Nuevo Kp: "); Serial.print(Kp);
    Serial.print(" | Ki: "); Serial.print(Ki);
    Serial.print(" | Kd: "); Serial.println(Kd);
  }

  delay(5);
}

void moveMotor(int pinPWM, int pinIN1, int pinIN2, float speed) {
  if (speed > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    analogWrite(pinPWM, -speed);
  }
}

void enableMotors() {
  digitalWrite(pinSTBY, HIGH);
}
