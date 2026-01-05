#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ----- OLED -----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----- DS18B20 -----
#define ONE_WIRE_BUS 18
OneWire onewire(ONE_WIRE_BUS);
DallasTemperature sensors(&onewire);
float temp = 0;

// ----- Серво -----
Servo myservo;
int servoPin = 19;

// ----- DHT11 -----
#define DHTPIN 25
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float roomTemp = 0;
float humidity = 0;

// ----- Потенциометър -----
#define POT_PIN 34

// ----- SERVO PWM -----
#define SERVO_STOP 1500
#define SERVO_MIN  1550
#define SERVO_50   1625
#define SERVO_MAX  1700

// ----- PID променливи -----
float Kp = 20.0, Ki = 0.05, Kd = 10.0;
float error = 0, previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

// ----- Adaptive PID -----
void adaptPID(float absError) {
  if (absError >= 5.0) {
    Kp = 30.0; Ki = 0.02; Kd = 15.0;
  }
  else if (absError >= 2.0) {
    Kp = 20.0; Ki = 0.05; Kd = 10.0;
  }
  else {
    Kp = 10.0; Ki = 0.1;  Kd = 5.0;
  }
}

int computePID(float setpoint, float measured) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.1;

  error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - previousError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  previousError = error;
  lastTime = now;

  output = constrain(output, 0, 200);
  return 1500 + output;
}

void setup() {
  Serial.begin(115200);

  sensors.begin();
  dht.begin();

  myservo.attach(servoPin, 500, 2500);

  Wire.begin(16, 17); // I2C за OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found");
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("DS18B20 READY");
  display.display();
  delay(1500);
}

void loop() {
  // ----- DS18B20 -----
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);

  // ----- DHT11 -----
  roomTemp = dht.readTemperature();
  humidity = dht.readHumidity();

  if (isnan(roomTemp) || isnan(humidity)) {
    roomTemp = 0;
    humidity = 0;
  }

  delay(3000);

  // ----- Потенциометър -----
  int potValue = analogRead(POT_PIN);
  float setTemp = 18.0 + (potValue / 4095.0) * 12.0;

  // ----- SERIAL PLOTTER -----
  /*
  Serial.print("SystemTemp:");
  Serial.print(temp);
  Serial.print(" ");

  Serial.print("RoomTemp:");
  Serial.print(roomTemp);
  Serial.print(" ");

  Serial.print("SetTemp:");
  Serial.print(setTemp);
  Serial.print(" ");

  Serial.print("Error:");
  Serial.println(setTemp - roomTemp);
  */

  // ----- OLED -----
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Set : ");  display.print(setTemp, 1); display.println(" C");
  display.print("Room: ");  display.print(roomTemp, 1); display.println(" C");
  display.print("Hum : ");  display.print(humidity, 0); display.println(" %");
  display.display();

  float sysError = setTemp - temp;
  float absSystemError = abs(sysError);
  float roomError = setTemp - roomTemp;
  float absRoomError = abs(roomError);

  // ---- Защита ----
  if (temp < setTemp - 5) {
    myservo.writeMicroseconds(SERVO_STOP);
  }

  // ---- Логическо управление ----
  else if (temp >= setTemp && roomTemp < setTemp) {
    if (absRoomError >= 5)
      myservo.writeMicroseconds(SERVO_MAX);
    else if (absRoomError > 2)
      myservo.writeMicroseconds(SERVO_50);
    else
      myservo.writeMicroseconds(SERVO_MIN);
  }

  else if (temp < setTemp && roomTemp < setTemp) {
    if (sysError <= 2)
      myservo.writeMicroseconds(SERVO_50);
    else
      myservo.writeMicroseconds(SERVO_MIN);
  }

  else if (roomTemp > setTemp) {
    myservo.writeMicroseconds(SERVO_STOP);
  }

  // ---- PID управление ----
  else {
    adaptPID(absRoomError);
    int pwm = computePID(setTemp, roomTemp);
    myservo.writeMicroseconds(constrain(pwm, SERVO_MIN, SERVO_MAX));
  }
}
