/***************************************************************************************
Autor:        Christian Magnus Obrecht
Projekt:      Bierkühler Steuer System HW test
Beschreibung: Büp bei Patrick Rupp Temperatursteuerung für ein Bierschnellkühlsystem Hardware test
Datum:        17.12.2023
Version:      1.0
****************************************************************************************/

#include <Arduino.h>
#include <Adafruit_MAX31865.h>      //Wird verwendet um den PT100 Sensor auszulesen
#include <Wire.h>                   //Wird verwendet um den I2C Bus zu verwenden
// put function declarations here:
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 13, 12); //software SPI: CS, DI, DO, CLK

#define RREF      4300.0                                       //Refferenz Widerstand

#define RNOMINAL  4500.0                                       //PT 100 Nomineller Widerstand


#define PWM_PUMPE 5               //PWM Pin der Pumpe
#define PWM_ROTATION 4            //PWM Pin des Rotationsmotors
#define PWM_PELTIER 8             //PWM Pin des Peltier Elements

#define PWM_MAX 1024              //Maximaler DutyCycle
#define PWM_OFF 0                 //Minimaler DutyCycle

#define BTN_START 21              //Pin des Start Knopfes


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcSetup(0, 20000, 10);
  ledcSetup(1, 20000, 10);
  ledcSetup(2, 20000, 10);
  ledcAttachPin(PWM_PUMPE, 0);
  ledcAttachPin(PWM_ROTATION, 1);
  ledcAttachPin(PWM_PELTIER, 2);
  pinMode(BTN_START, INPUT);
  }

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(BTN_START) == HIGH){
    Serial.println("Starting test");
    Serial.println(thermo.temperature(RNOMINAL, RREF));
    delay(1000);
    Serial.println("Press button to start PWM test");
    while(!digitalRead(BTN_START)){}
    Serial.println("Starting PWM test 20Khz at 100% duty cycle");
    ledcWrite(0, PWM_MAX);
    ledcWrite(1, PWM_MAX);
    ledcWrite(2, PWM_MAX);
    delay(1000);
    Serial.println("Press button for next PWM test");
    while(!digitalRead(BTN_START)){}

    Serial.println("Starting PWM test 20Khz at 50% duty cycle");
    ledcWrite(0, PWM_MAX/2);
    ledcWrite(1, PWM_MAX/2);
    ledcWrite(2, PWM_MAX/2);
    delay(1000);
    Serial.println("Press button for next PWM test");
    while(!digitalRead(BTN_START)){}

    Serial.println("Starting PWM test 20Khz at 0% duty cycle");
    ledcWrite(0, PWM_OFF);
    ledcWrite(1, PWM_OFF);
    ledcWrite(2, PWM_OFF);
  }
}


