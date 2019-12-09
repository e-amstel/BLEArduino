//ble
#include <SoftwareSerial.h>
//mpu6050
#include<Wire.h>
//leds
#include <Adafruit_NeoPixel.h>
//speaker
#include "pitches.h"


//#define MOTOR 3
#define PIN      5 //strip
#define N_LEDS 96 //amount of leds
  int c;
  int state = 0;
  int redLed = 0;
  int greenLed = 0;
  int blueLed = 0;

//--MPU6050--//
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int baseAccel;
int accel;
int lastAcc;
int diff; //container voor het berekenen van het verschil met de vorige meting. als te hoog: er wordt bewogen
int minDiff;
boolean sensorReact = true;   

 //--NEOPIXEL SETUP--//
// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, COLOR_WIPE, SPARKLE_WIPE, COLOR_RESET };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
    public:
    // Member Variables:  
    pattern  ActivePattern;  // which pattern is running    
    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position
    boolean init; //has the pattern been initialized/is it the first round
    
    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    uint16_t Index;  // current step within the pattern
    
    void (*OnComplete)();  // Callback on completion of pattern
    
    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
    :Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }
    
    // Update the pattern
    void Update()
    {
        if((millis() - lastUpdate) > Interval) // time to update
        {
            lastUpdate = millis();
            switch(ActivePattern)
            {
                case RAINBOW_CYCLE:
                    RainbowCycleUpdate();
                    break;
                case COLOR_WIPE:
                    ColorWipeUpdate();
                    break;
                case SPARKLE_WIPE:
                    SparkleWipeUpdate();
                    break;
                case COLOR_RESET:
                    ColorResetUpdate();
                    break;                   
                default:
                    break;
            }
        }
    }
  
    // Increment the Index and reset at the end
    void Increment()
    {
           Index++;
           if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
    }

    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
    }
    
    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval)
    {
        ActivePattern = COLOR_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Index = 0;
    }
    
    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        show();
        Increment();
    }
    
        // Initialize for a Wipe with a sparkle effect
    void SparkleWipe(uint32_t color, uint32_t endcolor, uint8_t interval)
    {
        ActivePattern = SPARKLE_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Color2 = endcolor;
        Index = 0;
    }
    
    // Update the Sparkle Wipe Pattern
    void SparkleWipeUpdate()
    {
        if (Index == 19 || Index == 39 || Index == 59 || Index == 71|| Index == 83 || Index == 95 ){
          setPixelColor(Index, Color2);
        }
        else {
          setPixelColor((Index+1), Color2);
          setPixelColor(Index, Color1);
        }
        show();
        Increment();
    }
        // Initialize for a reset pattern that colours the even and odd strips differently
    void ColorReset(uint32_t color, uint32_t color2, uint8_t interval, boolean start)
    {
        ActivePattern = COLOR_RESET;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Color2 = color2;
        Index = 0;
        init = start; 
    }
    
    // Update the Color reset Pattern
    void ColorResetUpdate()
    {
      if(init){
        if (Index == 0 || Index == 40 || Index == 72){
          setPixelColor(Index, Color2);
        }
        if (Index == 20 || Index == 60 || Index == 84){
          setPixelColor(Index, Color1);
        }
      }
      else{
        if (Index <= 19 || (Index >= 40 && Index <= 59) || (Index >= 72 && Index <= 83) ){
          setPixelColor(Index, Color2);
        }
        else {
          setPixelColor(Index, Color1);
        }
      } 
     show();
     Increment();
    }
  
    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }
 
    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if(WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};
 
void StripComplete();
 
NeoPatterns Strip(N_LEDS, PIN, NEO_GRB + NEO_KHZ800, &StripComplete);

//--DC--//
class Motor {
  int motorPin;
  long OnTime;
  long OffTime;
  int counter;
  int motorState;
  unsigned long previousMillis;
  boolean motorOn;

  public:
  Motor(int pin, long on){
    motorPin = pin;
    pinMode(motorPin, OUTPUT);
    OnTime = on;
    motorState = LOW;
    previousMillis = 0;
    counter = 0;
  }
  void Update() {
    unsigned long currentMillis = millis();
   // counter++;
   // Serial.println(previousMillis);
    
//
//    if (counter >= OnTime){
//      MotorOff();
//       Serial.println("low");

      if ((motorState == HIGH)&&(currentMillis - previousMillis) >= OnTime){ //als de tijd verlopen is uitzetten
        previousMillis  = currentMillis;
        MotorOff();
      }
      else {
       // MotorOn();

      }
   // }
  }    
   
    void MotorOff(){
      motorState = LOW;
      digitalWrite(motorPin, motorState);
      Serial.println("low");
      minDiff = 7;
      sensorReact = true;

    }
    
    void MotorOn(){
      motorState = HIGH;
      Serial.println("high");
      digitalWrite(motorPin, motorState);
      previousMillis = millis();
    // counter = 0;
      minDiff = 100;
      sensorReact = false; //om te voorkomen dat de sensor reageert op de motor die draait
    }
  
  
};

Motor motor(3,100);

//--SPEAKER--//

class Speaker {
  int melody[3];
  int noteDurations[3];
  int tonePin;
  unsigned long previousMillis;
  int counter;
  int numberOfNotes;
  boolean outputTone;

  public:
  Speaker(int pin){
    tonePin = pin;
    previousMillis = 0;
    counter = 0;
      melody[0] = NOTE_A6;
      melody[1] = NOTE_B6;
      melody[2] = NOTE_A7;
      noteDurations[0] = 4;
      noteDurations[1] = 4;
      noteDurations[2] = 4;

    numberOfNotes = 3;
  //      numberOfNotes = sizeof(melody)/sizeof(int);

    outputTone = false;
  }
  void StartMusic(){
    counter = 0;
  }
  void Update(){
  unsigned long currentMillis = millis();
  unsigned long noteDuration = 1000 / noteDurations[counter];
  unsigned long pauseBetweenNotes = 1000 / 4 ;

    if (counter < numberOfNotes) { //reset de counter  in een functie om de melodie opnieuw te beginnen 
      if (outputTone && ((currentMillis - previousMillis) >= noteDuration)) {
          previousMillis = currentMillis;
          noTone(tonePin);
          outputTone = false;
          counter++;
      }
      else {
          if ((currentMillis - previousMillis) >= pauseBetweenNotes) {
          previousMillis = currentMillis;
         // Serial.println(melody[counter]);
          if (melody[counter] == 0) {
            noTone(tonePin);
          } else {
            tone(tonePin, melody[counter]);
          }
          outputTone = true;
        }
      }
    }
  };
  
};

Speaker speaker(6);


//--BLE--//
SoftwareSerial mySerial(7, 8); // RX, TX  


// Initialize everything and prepare to start
void setup()
{
  Serial.begin(9600);
  // strips
    Strip.begin();
    Strip.setBrightness(64);
    Strip.ColorReset(Strip.Color(0, 0, 255), Strip.Color(125,0,125), 10, true); 

   // Strip.RainbowCycle(10);//startpattern
    
  //mpu6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  measureAccel();
  setBaseAccel();
  //ble
  mySerial.begin(9600);
  

}
 
// Main loop
void loop()
{
  
    Strip.Update();
    speaker.Update();
   // motor.Update();


    //read ble and set led
    checkBleInput();
    measureAccel();
    convertAccel();

           // Serial.println(diff);
     if ((sensorReact == true)&&(diff > 7)){
             speaker.StartMusic();
            // motor.MotorOn();
             Serial.println(diff);

     }

}
 
// Strip Completion Callback
void StripComplete()
{
    //Serial.println("eind strip");
    Strip.init = false;
}

//--BLE--//
void checkBleInput(){
  if (mySerial.available()) {
        Serial.println("Got input:");
        state = mySerial.read();
        Serial.println("State: "); Serial.println(state);
        redLed = mySerial.read();
        Serial.println(redLed);
        greenLed = mySerial.read();
        Serial.println(greenLed);
        blueLed = mySerial.read();
        Serial.println(blueLed);
        if(state == 0){Strip.RainbowCycle(10);};
        if(state == 1){Strip.ColorReset(Strip.Color(redLed, greenLed, blueLed), Strip.Color(125,0,125),10, true);};
        if(state == 2){Strip.ColorWipe(Strip.Color(redLed, greenLed, blueLed),20);};
        if(state == 3){Strip.SparkleWipe(Strip.Color(redLed, greenLed, blueLed), Strip.Color(125,125,125), 30);};
        
     //   speaker.StartMusic();
       //     motor.MotorOn();

  }   
  else {
      // Serial.println("noInput");
  }
} 
//--MPU6050--//
void measureAccel (){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
 // delay(30);
}

void setBaseAccel (){
    baseAccel = (AcZ - 15000)/1000;
    Serial.print(" Base =  ");Serial.print(baseAccel);
}
void convertAccel (){
  accel = (AcZ - 15000)/1000;  
  accel = accel - baseAccel;
  if (accel < 0){ accel = -accel; }   //tuning
 // Serial.print(" Accel =  ");Serial.print(accel);
  diff = accel - lastAcc;
//Serial.print(" Difference =  ");Serial.print(diff);
  lastAcc = accel;
}
