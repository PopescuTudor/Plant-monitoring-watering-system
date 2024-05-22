#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <AHTxx.h>

LiquidCrystal_I2C lcd(0x27,16,2);  
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type

const int trigPin = 8;
const int echoPin = 2;
const int PUMP_PIN = 3;
const int BUZZER_PIN = 9;
const int SOIL_PIN = 0;
volatile bool isWaterLow = false;
float ahtValue;                               //to store T/RH result
volatile int soilHumidity = 0;


#define max_distance 30 // cm

void setup() {
  Serial.begin(9600);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize LCD
  lcd.init();                      
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.setCursor(7, 0);
  lcd.print("C");
  lcd.print(" H:");
  lcd.setCursor(0,1);
  lcd.print("Nivel apa:");

  // Attach interrupt for echo pin
  attachInterrupt(digitalPinToInterrupt(echoPin), waterLevelInterrupt, CHANGE);

  // Enable PIN CHANGE interrupts for port C
  PCICR |= B00000010;
  // Enable pin A0 by modifying the pin change mask
  PCMSK1 |= B00000001; 
}

ISR(PCINT1_vect) {
  if(analogRead(A0) > soilHumidity) {
    soilHumidity = analogRead(A0);
    startPump(255);
  } else {
    soilHumidity = analogRead(A0);
    startPump(0);
  }
}

void startPump(int speed) {
  // Start the pump with PWM
  analogWrite(PUMP_PIN, speed);
}

void waterLevelInterrupt() {
  if (isWaterLow == true) {
    analogWrite(PUMP_PIN, 0);  
    tone(BUZZER_PIN, 300, 100);

  } 
  else {
    // startPump(255);
    noTone(BUZZER_PIN);

  }
}

float measureDistance() {
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the duration of the echo
  long duration = pulseIn(echoPin, HIGH);
  
  // Convert the duration to distance in cm
  float distance = duration * 0.034 / 2; // Speed of sound is approximately 34 cm/ms
  
  return distance;
}

void loop() {
  long distance = 0;
  ahtValue = aht10.readTemperature(); //read 6-bytes via I2C
  if(ahtValue != AHTXX_ERROR) {
    lcd.setCursor(2, 0);
    lcd.print(ahtValue);
  }
  ahtValue = aht10.readHumidity();
  distance = measureDistance();
  if(ahtValue != AHTXX_ERROR) {
    lcd.setCursor(11, 0);
    lcd.print(ahtValue);
    lcd.setCursor(15, 0);
    lcd.print("%");
  }
  lcd.setCursor(11,1);
  lcd.print("      ");
  lcd.setCursor(11,1);

  
  if(distance <4) {
    lcd.print("HIGH");
    isWaterLow = false;
  } else if (distance >= 4 && distance <=7) {
    lcd.print("MID");
    isWaterLow = false;
  } else {
    lcd.print("LOW");
    isWaterLow = true;
  }
  delay(300);
}
