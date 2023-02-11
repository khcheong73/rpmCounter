// include the library code:
#include <Arduino.h>
#include <U8g2lib.h>
#include <Servo.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

//------code for RPM----------------
int encoderPIN = 2; // The pin the encoder is connected
unsigned int rpm; // rpm reading
volatile byte pulses; // number of pulses
unsigned long timePrev; // The number of pulses per revolution. depends on your index disc!!
unsigned int pulsesPerTurn = 1;

// added by Kevin
Servo myServo;
int numBlade = 1;
int setModePIN = 4;
int setBladePIN = 3;
int aInputPIN = A6;
int servoVolPIN = A3;
float servoVol;
float aInput;
int aInputINT;
float sAngle;
int sAngleINT;
int pwmPIN=6;

int Mode;

void counter()
{
    //Update count
    pulses++;
}
//-----------------------------------
void setup()   {
    Serial.begin(9600);
    u8g2.begin();
    myServo.attach(pwmPIN);
    
//-----------code for RPM---------------
    //Use statusPin to flash along with interrupts
    pinMode(encoderPIN, INPUT);
    //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
    //Triggers on FALLING (change from HIGH to LOW)
    attachInterrupt(digitalPinToInterrupt(encoderPIN), counter, FALLING);
    attachInterrupt(digitalPinToInterrupt(setBladePIN), count_blade, FALLING);
    // Initialize
    pulses = 0;
    rpm = 0;
    timePrev = 0;
    
    pinMode(setBladePIN, INPUT_PULLUP);
    pinMode(setModePIN, INPUT);
}

void readRPM() {
  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
  //Don't process interrupts during calculations
  detachInterrupt(digitalPinToInterrupt(encoderPIN));
  detachInterrupt(digitalPinToInterrupt(setBladePIN));
  //Note that this would be 60*1000/(millis() - timePrev)*pulses if the interrupt
  //happened once per revolution
  rpm = (30 * 2000 / pulsesPerTurn / numBlade)/ (millis() - timePrev)* pulses;
  timePrev = millis();
  pulses = 0;

  //Write it out to serial port 
  //Serial.print("NUM BLADE = ");
  //Serial.println(numBlade);
  //Serial.print("RPM = "); 
  //Serial.println(rpm, DEC);
  u8g2.drawStr(0,20,"NUM BLADE  = ");
  u8g2.setCursor(80,20); u8g2.print(numBlade);
  u8g2.drawStr(0,30,"Rotate/min = ");
  u8g2.setCursor(80,30); u8g2.print(rpm);
}

void loop()
{
  Mode = digitalRead(setModePIN);
  switch (Mode) {
    case 0: // Servo test mode
      if (analogRead(servoVolPIN) > 850 ) {
        servoVol=5.0;
      }
      else servoVol=3.3;
      aInput=analogRead(aInputPIN);
      sAngle=(aInput/1023)*180;
      sAngleINT=sAngle;
      aInputINT=aInput;
      myServo.write(sAngle);
      
      u8g2.clearBuffer();              // clear the internal memory
      u8g2.setFont(u8g2_font_6x12_tr); // choose a suitable font
      u8g2.drawStr(0,8,"Servo Mode          V");  // write something to the internal memory
      u8g2.setCursor(96,8); u8g2.print(servoVol);
      u8g2.drawHLine(0,10,128);
      u8g2.drawStr(0,20,"A.Input =");
      u8g2.setCursor(60,20); u8g2.print(aInputINT);
      u8g2.drawStr(0,30,"S.Angle =     deg.");
      u8g2.setCursor(60,30); u8g2.print(sAngleINT);
      u8g2.sendBuffer();          // transfer internal memory to the display
      delay(100);  
      break;
    case 1: // RPM counter mode
      if (millis() - timePrev >= 2000){
        u8g2.clearBuffer();          // clear the internal memory
        u8g2.setFont(u8g2_font_6x10_tr); // choose a suitable font
        u8g2.drawStr(0,8,"RPM Counter Mode");  // write something to the internal memory
        u8g2.drawHLine(0,10,128);
        readRPM();
     
        //Restart the interrupt processing
        attachInterrupt(digitalPinToInterrupt(encoderPIN), counter, FALLING);
        attachInterrupt(digitalPinToInterrupt(setBladePIN), count_blade, FALLING);
      }

      u8g2.sendBuffer();          // transfer internal memory to the display
      delay(1000);  
      break;
  }
    
}

void count_blade()
{
  numBlade++;
  if (numBlade > 4) {
    numBlade = 1;
  }
}