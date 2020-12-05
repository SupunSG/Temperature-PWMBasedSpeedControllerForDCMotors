#include "DHT.h"
#include <Adafruit_Sensor.h>
#include "Plotter.h"
#include <LiquidCrystal.h>

#define DHTPIN 7  // digital pin which is connected to the temperature sensor
#define DHTTYPE DHT21   // DHT 21 (AM2301) temperature sensor

DHT dht(DHTPIN, DHTTYPE);     // Initialize DHT sensor.
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int EN_A = 10;      //Enable pin for motor
int IN1 = 9;       //control pin for motor
int IN2 = 8;       //control pin for motor
int motorSpeed;
double x;   //variables for graph
double y;
Plotter p;

void setup() {
  Serial.begin(115200);
  Serial.println("DHT21 test!");
  dht.begin();

  lcd.begin(16, 2);
  lcd.print("DHT test!");
  delay(2000);

  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);

  p.Begin();
  p.AddTimeGraph("TEMPERATURE AND MOTOR SPEED VS TIME GRAPH", 500, "TEMPERATURE", x, "MOTOR SPEED", y );
  p.SetColor( 0, "blue", "yellow" );
}

void loop() {
  float temperature = getTemperature();   // To read temperature
  int motorSpeed=getMotorSpeed(temperature); //Convert values to motor speed
  driver(motorSpeed);                             // To drive the motor
  printData(temperature,motorSpeed);    // To print data
  graphPlotter(temperature, motorSpeed);    // To plot the two graphs
  delay(50);
}

float getTemperature(){
   return dht.readTemperature();    // Read temperature as Celsius 
}

float getMotorSpeed(float temp){    // Converting values
  return map(temp, 0,40,0,255);
}

void driver(int motorSpeed){      // Drive the motor
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN_A, motorSpeed);
}
  
void printData(float temperature, int motorSpeed){
  lcd.clear();
  lcd.print("Temperature: ");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(0,2);
  lcd.print("Motor Speed: ");
  lcd.print(motorSpeed);
}
  
void graphPlotter(float temperature, float motorSpeed
  x = temperature;
  y = motorSpeed;
  p.Plot();
}

