/***************************************************************************************
Autor:        Christian Magnus Obrecht
Projekt:      Bierkühler Steuer System
Beschreibung: Büp bei Patrick Rupp Temperatursteuerung für ein Bier schnellkühlsystem
Datum:        28.11.2023
Version:      1.0
****************************************************************************************/

/***************************************************************************************
Autor:        Aris Cadruvi
Projekt:      Bierkühler Webserver
Beschreibung: Büp bei Patrick Rupp Bierkühler steuer und Überwachungs Webserver
Datum:        28.11.2023
Version:      1.0
****************************************************************************************/

//Die beiden seperaten Programme werden auf die 2 Cores auf einem ESP32 aufgeteilt
//Die einzelnen Softwaren weden getrennt geführt und es wird beschriben welcher Code von wem ist

#include <Arduino.h>                //Wird verwendet um Arduino befehle zu verwenden
#include <esp_task_wdt.h>           //Wird verwendet für das Multicore setup
#include <esp_sleep.h>              //Wird verwendet um den ESP in deepSleep zu versetzen
#include <driver/rtc_io.h>          //Wird verwendet um den Taster als Wakeup Pin zu deffinieren
#include <Adafruit_MAX31865.h>      //Wird verwendet um den PT100 Sensor auszulesen
#include <Adafruit_NeoPixel.h>      //Wird verwendet um die RGB LED zu steuern
#include <SPI.h>                    //Wird verwendet für den SPI bus
#include <Wire.h>                   //Wird verwendet für den I2C bus
#include <EEPROM.h>                 //Wird verwendet um daten über das gerät zu speichern

//****************************************************//
//Deklarationen
//Christian Magnus Obrecht
//****************************************************//

//Einstellungen des Systems
//Temperatur in °C
#define START_TEMP 4              //Temperatur bei welcher der Bierkühler einsatzbereit ist
#define MIN_TEMP 2                //Ausschaltschwelle der Kühlung

#define LAUFZEIT 60               //Bier kühl Zeit in s
#define STANDBYZEIT 30            //Zeit nach welcher der Kühler ausschaltet in s
#define LANGERDRUCK 4             //Zeit länge eines langen Knopfdruckes in s
#define NOM_SPEED_ROTATION 60     //DutyCycle des Rotationsmotors 0-1024
#define NOM_SPEED_PUMPE 1000      //DutyCycle der Pumpe 0-1024
#define PWM_PUMPE 5               //PWM Pin der Pumpe
#define PWM_ROTATION 4            //PWM Pin des Rotationsmotors
#define PWM_PELTIER 8             //PWM Pin des Peltier Elements
#define BTN_START 21              //Pin des Start Knopfes
#define PWM_MAX 1024              //Maximaler DutyCycle
#define PWM_OFF 0                 //Minimaler DutyCycle

//Pins für NeoPixel
#define NEOPIXEL_PIN 14           //Pin des NeoPixels
#define NUMPIXELS 64              //Anzahl der NeoPixels

//Modi des Systems
#define OFF 0                     //ESP wechselt in deepSleep
#define STANDBY 1                 //ESP wartet auf Startsignal oder auf Ablauf des standbytimers
#define COOLDOWN 2                //ESP startet die Vorkühlung bis die Temperatur START_TEMP erreicht wird oder der Taster lang gedrückt wird
#define COOLING 3                 //ESP startet die Kühlung für LAUFZEIT

//EEPROM deffinitionen
#define EEPROM_SIZE 3             //Grösse des EEPROM Speichers in byte
#define EEPROM_ERROR 0            //Adresse des Error Counters
#define EEPROM_BIER_MSB 1         //Adresse des Bier Counters
#define EEPROM_BIER_LSB 2         //Adresse des Bier Counters

//Timer Zeiten in Millisekunden
uint16_t LangerDruckInMills = LANGERDRUCK * 1000;
uint16_t LaufzeitInMills = LAUFZEIT * 1000;
uint16_t StandbyzeitInMills = STANDBYZEIT * 1000;

//Faktor für Neopixel pro sekunde
uint8_t NeoPixelFaktor = 0;

//Daten für den EEPROM
uint8_t ErrorCnt = 0;             //Speicher für Anzahl der Fehler
uint16_t BierCnt = 0;             //Speicher für die Anzahl der gekühlten Biere

//Sperre für den Taster posFlanke
bool ButtonLockout = false;

uint8_t mode = 1;                 //ESP startet im Standby

float aktueleTemp = 0;            //Aktuelle Temperatur
uint16_t restzeit = 0;            //Restzeit der Kühlung
unsigned long Lauftimer = 0;      //Timer für die Laufzeit
unsigned long Standbytimer = 0;   //Timer für die Standbyzeit
unsigned long Drucktimer = 0;     //Timer für den Taster

//****************************************************//
//Multicore Task Setup
//Christian Magnus Obrecht
//****************************************************//

//Task Deklarationen
void RUNTIME (void * pvParameters); 
void WEBSRV (void * pvParameters);

//Task Handles
TaskHandle_t Task1;
TaskHandle_t Task2;


//****************************************************//
//Temperatur Sensor Setup
//Christian Magnus Obrecht
//****************************************************//

//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 13, 12); //software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(36, 6, 5, 4); //software SPI: CS, DI, DO, CLK
#define RREF      220.0                                       //Refferenz Widerstand

#define RNOMINAL  100.0                                       //PT 100 Nomineller Widerstand


//****************************************************//
//Arduino Setup
//Wird nur für TASK initialisierung verwendet
//Christian Magnus Obrecht
//****************************************************//

void setup() {

  

  //Task 1 wird auf Core 0 gestartet und Regelt den gesamten Ablauf
  xTaskCreatePinnedToCore(
                    RUNTIME,     /* Task Funktions Name */
                    "Task1",     /* Task Name */
                    10000,       /* Stack grösse */
                    NULL,        /* Keine Parameter werden übergeben */
                    1,           /* Priorität des Taskes */
                    &Task1,      /* Task Handle um Task zu verwalten */
                    0);          /* TASK läuft auf Core 0 */
  delay(1);                      // Kurzer Wait um sicherzustellen das der Task gestartet ist

//Task 2 wird auf Core 1 gestartet und Regelt den Webserver(BüP Aris Cadruvi)
  xTaskCreatePinnedToCore(
                    WEBSRV,      /* Task Funktions Name */
                    "Task2",     /* name of task. */
                    10000,       /* Stack grösse */
                    NULL,        /* Keine Parameter werden übergeben */
                    1,           /* Priorität des Taskes */
                    &Task2,      /* Task Handle um Task zu verwalten */
                    1);          /* TASK läuft auf Core 0 */
  delay(1);                      // Kurzer Wait um sicherzustellen das der Task gestartet ist
}

//****************************************************//
//Gesamte Steuerung des Kühlsytems
//Christian Magnus Obrecht
//****************************************************//
void RUNTIME( void * pvParameters ){

//****************************************************//
//Setup des Steuerungssystems
//Christian Magnus Obrecht
//****************************************************//

  //EEPROM initialisieren
  EEPROM.begin(EEPROM_SIZE);

  //NeoPixel initialisieren
  Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
  pixels.begin();
  pixels.clear();
  //Daten aus dem EEPROM lesen
  ErrorCnt = EEPROM.read(EEPROM_ERROR);
  BierCnt = EEPROM.read(EEPROM_BIER_MSB) << 8 | EEPROM.read(EEPROM_BIER_LSB);

  //Startknopf als Eingang deffinieren
  rtc_gpio_deinit(GPIO_NUM_21);
  pinMode(BTN_START, INPUT);

  //PWM channel einrichten 10Bit @20kHz
  ledcSetup(0, 20000, 10);
  ledcSetup(1, 20000, 10);
  ledcSetup(2, 20000, 10);

  //PWM channel an Pin binden
  ledcAttachPin(PWM_PUMPE, 0);
  ledcAttachPin(PWM_ROTATION, 1);
  ledcAttachPin(PWM_PELTIER, 2);

  //Serial Monitor starten (Aktuell nur für Debugging)
  Serial.begin(115200);

  //PT100 initialisieren
  thermo.begin(MAX31865_2WIRE);  //läuft im 2 Draht Modus

  //StandbyTimer starten
  Standbytimer = millis ();

  Serial.print("Error Count: ");
  Serial.println(ErrorCnt);
//****************************************************//
//Endlosschleife des Steuerungssystems
//Christian Magnus Obrecht
//****************************************************//
  for(;;){
    esp_task_wdt_init(5, true); //Watchdog Timer wird gestartet
    //Aktuelle Temperatur auslesen
    aktueleTemp = thermo.temperature(RNOMINAL, RREF);

    //Modes in switch case
    switch (mode){

      //Peripherien werden Abgeschaltet und ESP wechselt in deepSleep
      case OFF:
        Serial.println("Going to sleep");
        //Wakeup Interupt Pin wird deffiniert
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, 1);
        mode = STANDBY;
        esp_deep_sleep_start();  //ESP wechselt in deepSleep
      break;

      //ESP wartet auf Startsignal oder auf Ablauf des standbytimers
      case STANDBY:
        if(!digitalRead(BTN_START)){
          ButtonLockout = false;
        }
        //Programm wechselt in OFF MODE
        if(Standbytimer + StandbyzeitInMills <= millis()){
          mode = OFF;
       }
       
        //NeoPixel werden auf rot gesetzt und die Restzeit wird angezeigt
        restzeit = (StandbyzeitInMills - (millis() - Standbytimer)) / 1000;   //Restzeit wird berechnet
        if(restzeit > STANDBYZEIT){                                           //Wenn die Restzeit grösser als die Standbyzeit ist wird die Restzeit auf 0 gesetzt
          restzeit = 0;
        }
        NeoPixelFaktor = 64 / STANDBYZEIT;                                    //Faktor für die Anzahl der NeoPixel pro Sekunde wird berechnet       
        pixels.clear();                                                       //Alle NeoPixel werden ausgeschaltet       
        for(int i = 0; (i < NeoPixelFaktor * restzeit); i++){                 //Die Anzahl der NeoPixel wird berechnet und eingeschaltet
          pixels.setPixelColor(i, pixels.Color(0, 50, 0));
        }
        pixels.show();                                                        //NeoPixel werden aktualisiert        

       //Alle Endstufen werden abgeschaltet
        for(int i = 0; i <= 2; i++){
          ledcWrite(i, 0);
        }
        //Bei Knopfdruck wird in die Vorkühlung gewechselt
        if(digitalRead(BTN_START)&& ButtonLockout == false){
          Serial.println("Button pressed Cool down");
          delay(100);
          mode = COOLDOWN;
          pixels.clear();                                                       //Alle NeoPixel werden ausgeschaltet  
          pixels.show();                                                        //NeoPixel werden aktualisiert
        }
      break;

      //ESP startet die Vorkühlung bis die Temperatur START_TEMP erreicht wird oder der Taster lang gedrückt wird
      case COOLDOWN:
        Serial.println("Cooling down");
        //Wenn die Temperatur über MIN_TEMP ist werden die Pumpe und die Peltier elemente eingeschaltet
        if(aktueleTemp >= MIN_TEMP){
          ledcWrite(0, NOM_SPEED_PUMPE);
          ledcWrite(2, PWM_MAX);
          Serial.print("Cooling down, Current Temp: ");
          Serial.println(aktueleTemp);
        }

        //Temperatur ist unter MIN_TEMP und die Peltier Elemente und die Pumpe werden ausgeschaltet
        else{
          ledcWrite(0, PWM_OFF);
          ledcWrite(2, PWM_OFF);
          Serial.println("Target temp reached");
        }
        //Wenn der Knopf eine pos Flanke sieht wird startet der drucktimer und Lockout wird gesetzt
        if(digitalRead(BTN_START) && ButtonLockout == false){
          ButtonLockout = true;
          Drucktimer = millis();
        }
        //Wenn der Knopf losgelassen wird bevor die Lang druck Zeit abgelaufen ist wird der Kühlvorgang gestartet
        if((!digitalRead(BTN_START)) && ButtonLockout == true){
          ButtonLockout = false;
          mode = COOLING;
          Serial.println("Starting Cooling");
          Lauftimer = millis();

     
        }
        //Wenn der Knopf gedrücktbleibt bis die Lang druck Zeit abgelaufen ist wird in den Standby Modus gewechselt
        if(digitalRead(BTN_START) && ButtonLockout == true && Drucktimer + LangerDruckInMills < millis()){
          Serial.println("Button pressed Standby");
          mode = STANDBY;
          Standbytimer = millis ();
        }
      break;
      //ESP startet die Kühlung für LAUFZEIT und der Rotationsmotor wird eingeschaltet
      case COOLING:
        Serial.println("Cooling");
        //Während der Laufzeit wird die Pumpe und der Rotationsmotor eingeschaltet und die Temperatur überwacht
        if(Lauftimer + LaufzeitInMills >= millis()){
          restzeit = (LaufzeitInMills - (millis() - Lauftimer)) / 1000;
          ledcWrite(0, NOM_SPEED_PUMPE);
          ledcWrite(1, NOM_SPEED_ROTATION);
          Serial.print("Cooling, Current Temp: ");
          Serial.print(aktueleTemp);
          Serial.print(" Restzeit: ");
          Serial.println(restzeit);

          //NeoPixel werden auf rot gesetzt und die Restzeit wird angezeigt
          NeoPixelFaktor = 64 / LAUFZEIT;                                       //Faktor für die Anzahl der NeoPixel pro Sekunde wird berechnet       
          pixels.clear();                                                       //Alle NeoPixel werden ausgeschaltet       
          for(int i = 0; (i < NeoPixelFaktor * restzeit); i++){                 //Die Anzahl der NeoPixel wird berechnet und eingeschaltet
            pixels.setPixelColor(i, pixels.Color(0, 0, 50));
          }
          pixels.show();                                                        //NeoPixel werden aktualisiert        

          if(aktueleTemp >= MIN_TEMP){
            ledcWrite(2, PWM_MAX);
          }
          else{
            ledcWrite(2, PWM_OFF);
          }
        }
        //Wenn die Laufzeit abgelaufen ist wird in den COOLDOWN Modus umgeschaltet
        else{
          BierCnt++;
          EEPROM.write(EEPROM_BIER_MSB, BierCnt >> 8);
          EEPROM.write(EEPROM_BIER_LSB, BierCnt);
          EEPROM.commit();
          Serial.print("Cooling done, cooled Beers: ");
          Serial.println(BierCnt);
          ledcWrite(0, 0);
          ledcWrite(1, 0);
          ledcWrite(2, 0);
          mode = COOLDOWN;
        }
        break;
      //Falls der Mode nicht definiert ist wird in den OFF Mode gewechselt was einen Reset des ESP auslöst und den Fehler beheben sollte
      default:
        ErrorCnt++;
        EEPROM.write(EEPROM_ERROR, ErrorCnt);
        EEPROM.commit();
        mode = OFF;
      break;
    }
  }
}
//****************************************************//
//Gesamte Steuerung des Webservers
//Code von Aris Cadruvi
//****************************************************//
void WEBSRV( void * pvParameters ){

//****************************************************//
//Setup des Webservers
//Aris Cadruvi
//****************************************************//

/*Code Here*/


//****************************************************//
//Endlosschleife des Webservers
//Aris Cadruvi
//****************************************************//
  for(;;){
  esp_task_wdt_init(5, true); //Watchdog Timer wird gestartet
  /*Code Here*/
  }
}

//****************************************************//
//Arduino Loop darf nicht verwendet werden
//Christian Magnus Obrecht
//****************************************************//
void loop() {

}