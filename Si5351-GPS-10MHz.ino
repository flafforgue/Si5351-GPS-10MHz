// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//  0    : GPS    - Tx ( alternate )
//  1    :      
//  2    : GPS    - 1 PPS
//  3    : Rotary A
//  4    : Rotary B
//  5    : Si5351 - Clk0 2.5 MHz
//  6    : Tft    - 
//  7    : Tft    -
//  8    : GPS    - Tx  ( Base )     
//  9    : GPS    - Rx  ( Not connected )
// 10    :
// 11    : Tft    -
// 12    :
// 13    : Tft    -
// 14 A0 :
// 15 A1 :
// 16 A2 :        - Led
// 17 A3 : Dalas  - D18B20
// 18 A4 : Si5351 - SDA
// 19 A5 : Si5351 - SCL
// 20 A6 : Rotary - Btn
// 21 A7 :

// ============================================================================
//                          OLED 240 x 240 Screen
// ============================================================================

#include <SPI.h>             // Arduino SPI library
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(-1, 6, 7);

// ----------------------------------------------------------------------------

void InitTft() {
  tft.init(240, 240, SPI_MODE2); 
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_YELLOW);  
}

// ============================================================================
//                                 ---   GPS   ---
// ============================================================================

#include <TimeLib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 9;
static const int TXPin = 8;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// ----------------------------------------------------------------------------

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

// ----------------------------------------------------------------------------
byte oday = 0;
byte osec = 61;

void GPSTimeDisplay(){
  if ( gps.date.isUpdated() ) {
    int  Year   = gps.date.year();
    byte Month  = gps.date.month();
    byte Day    = gps.date.day();
    byte Hour   = gps.time.hour();
    byte Minute = gps.time.minute();
    byte Second = gps.time.second();

    setTime(Hour, Minute, Second, Day, Month, Year);
    adjustTime( 2 * SECS_PER_HOUR);        
  }
  
  char sz[32];
  if ( oday != day() ) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(3);
    tft.fillRect( 5,20, 18*10,24, ST77XX_BLACK);
    tft.setCursor(5,20);
    sprintf(sz, "%02d/%02d/%02d ", day(), month(), year() );
    tft.print(sz);
    oday = day();
  }

  if ( osec != second() ) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(3);
    tft.fillRect( 5,50, 18*10,24, ST77XX_BLACK);
    tft.setCursor(5,50);
    sprintf(sz, "%02d:%02d:%02d ", hour(), minute(), second() );
    tft.print(sz);
    osec = second();
  }
  
} 

// ----------------------------------------------------------------------------
 
void InitGps() {
  ss.begin(GPSBaud);  
}

// ============================================================================
//                      Dalas D18B20 Temperature Sensor
// ============================================================================

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(A3);
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer;

void InitThermometer() {
  sensors.begin();
  if (!sensors.getAddress(Thermometer, 0)) tft.print(" Error !") ;
  sensors.setResolution(Thermometer, 9);
}

// ============================================================================
//                               ---   Rotary   ---
// ============================================================================

#define encoder0PinA      3 
#define encoder0PinB      4 
#define BtnEncoder       20
#define BTN_None          0
#define BTN_Encoder       1
#define BTN_EncoderL      2
#define BTN_LONGDELAY   800

int posencoder          = 0;
int encodermov          = 0;
unsigned long ntime     = 0;
unsigned long otime     = 0;

byte keydown            = BTN_None;
byte key                = BTN_None;
unsigned long BTNTime;

// ----------------------------------------------------------------------------

void doEncoder() {
  ntime=millis();
  if ( ntime-otime > 60 ) {
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
      encodermov =+1;
      posencoder++;
    } else {
      encodermov =-1;
      posencoder--;
    }
    otime=ntime;
  }
}

// ----------------------------------------------------------------------------

void readBtnState() {
  unsigned long NTime; 
  NTime=millis();
  
  if ( keydown == BTN_None ) { // no key waiting 
    if ( digitalRead (BtnEncoder)==LOW ) { BTNTime=NTime;  keydown=BTN_Encoder;  }
//    if ( digitalRead (BtnCh1    )==LOW ) { BTNTime=NTime;  keydown=BTN_Ch1;      }
//    if ( digitalRead (BtnCh2    )==LOW ) { BTNTime=NTime;  keydown=BTN_Ch2;      }    
  } else {                     // key allready down
    if ( NTime - BTNTime > 10 ) { // avoid rebounds
        switch (keydown) {
           case BTN_Encoder:
                 if ( digitalRead (BtnEncoder)==HIGH ) { // keypress on release ;)   
                   if ( NTime - BTNTime >= BTN_LONGDELAY )  key = BTN_EncoderL;                
                   else                                     key = BTN_Encoder;
                   keydown=BTN_None;
                 }
                 break;   
        }       
    }
  }
}

// ----------------------------------------------------------------------------

bool keypressed() {
  return ( key != BTN_None );
}

byte readkey() {
  byte tmp = key;
  key=BTN_None;
  return( tmp);
}

// ----------------------------------------------------------------------------

void InitRotary() {
  // encoder  
  // ----------
  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE); 
  
  // Boutons 
  // ----------
  pinMode(BtnEncoder  , INPUT_PULLUP);

}

// ============================================================================
//                             ---   Frequency   ---
// ============================================================================

#include <Wire.h>
#include "Si5351.h"

#define PPS_pin 2
#define FIn_Pin 5

boolean FrequencyUpdatePending = false;
double  CorrectionFactor       =  1.0;
long    Freq_2_5_Mhz           =  80000000 ;  //  2500000 * 32;  // 2.5 MHz * 32 ( Final divider ) = 80 MHz
long    Freq_10_Mhz            =  80000000 ;  // 10000000 *  8;  //  80 Mhz with Final divider 8   = 10 Mhz
long    Freq_1_Mhz             =  64000000 ;  //  1000000 * 64; 

long    Old_2_5_Mhz            =  0;
long    Old_10_Mhz             =  0;
long    Old_1_Mhz              =  0;

#define  count_error              2              // counter error due to Test function and additional instruction
                                                 // value to be removed from counter value

#define counterdelay              4              // delay in seconds to start counter
int     timegate               = 64;             // duration of timegate ( + counterdelay )
 
// ----------------------------------------------------------------------------
//                                Timer 1 overflow 
// ----------------------------------------------------------------------------

long ovfcpt = 0;

ISR( TIMER1_OVF_vect ) {
  ovfcpt++;                        // Increment multiplier
  TIFR1 = ( 1 << TOV1 );           // Clear overlow flag 
}

// ----------------------------------------------------------------------------
//                                   PPS pulse
// ----------------------------------------------------------------------------

unsigned long PPSCount = 0;
unsigned long Fcounter = 0;

void doPPS() {
  PPSCount++;
  if ( PPSCount == counterdelay ) {
    TCCR1B   = 7;
  } else if ( PPSCount == timegate ) {
    TCCR1B   = 0;                          // Turn off counter  
    Fcounter = ovfcpt * 0x10000 + TCNT1;
    TCNT1    = 0;
    ovfcpt   = 0;
    PPSCount = 0;
    FrequencyUpdatePending=true;
  }
}

// ----------------------------------------------------------------------------

void InitFrequency() {
  noInterrupts();
 
  pinMode(PPS_pin, INPUT);
  pinMode(FIn_Pin, INPUT);

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  
  attachInterrupt(digitalPinToInterrupt(PPS_pin), doPPS, RISING ); 
  interrupts();
}

// ----------------------------------------------------------------------------

double OldCorrectionFactor = 9.0;

void UpdateFrequency() {

    Serial.println("----------");   
    Serial.println(Fcounter); 
    Serial.println(Freq_2_5_Mhz);
            
    noInterrupts();
    CorrectionFactor = ( 2500000.0 * ( timegate - counterdelay ) ) / ( Fcounter - count_error );  // Target Frequency x Gate Time / Actual Counter value
    PPSCount         = 0;
    interrupts(); 
    
    Freq_2_5_Mhz         = long( Freq_2_5_Mhz * CorrectionFactor + 0.5 );
    Freq_10_Mhz          = long( Freq_10_Mhz  * CorrectionFactor + 0.5 );
    Freq_1_Mhz           = long( Freq_1_Mhz   * CorrectionFactor + 0.5 );    

    Serial.print((1.0 - CorrectionFactor )*1000000.0);
    Serial.println(" ppm");    
    Serial.println(Freq_2_5_Mhz);
    Serial.println(Freq_10_Mhz); 

    float tempC = sensors.getTempC(Thermometer);
    Serial.print  (tempC);
    Serial.println(" °C"); 
}

// ============================================================================
//                               ---   Setup   ---
// ============================================================================

void setup(void) {
  InitTft();
  InitThermometer();
  InitRotary();
  InitGps();
  InitFrequency();

  Wire.begin();
  Si5351_Init();
  Si5351_SetFreq(SYNTH_MS_0, Freq_2_5_Mhz ,RDIV_32 );
  Si5351_SetFreq(SYNTH_MS_1, Freq_10_Mhz  ,RDIV_8 );
  Si5351_SetFreq(SYNTH_MS_2, Freq_1_Mhz   ,RDIV_64 );    
  Si5351_write(CLK_ENABLE_CONTROL,0b00000100); // Clk2 disable


  tft.setTextSize(2);
  tft.setCursor(0,10);
  tft.print("Waiting for GPS");
  smartDelay(1000);

  while ( gps.charsProcessed() < 10 ) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(10, 110);
      tft.setTextSize(3);
      tft.print("No GPS Found !");
      smartDelay(1000);
  }
  tft.fillScreen(ST77XX_BLACK);

  tft.setCursor(0,10);
  tft.print("Waiting for Time");
  while ( not(gps.time.isValid()) ) {
      smartDelay(500);
  }
  
  tft.setCursor(0,30);
  tft.print("Waiting for GPS");
  while ( not(gps.location.isValid()) ) {
      smartDelay(500);
  }
    
  noInterrupts();
  TCCR1B   = 0;
  Fcounter = 0;
  TCNT1    = 0;
  ovfcpt   = 0;
  PPSCount = 0;
  interrupts();
  tft.fillScreen(ST77XX_BLACK);  

  Serial.begin(115200);
  Serial.println("GPS Ready");    
}

// ============================================================================
//                              ---   Main Loop   ---
// ============================================================================

int ActSel=0;

long   DisplayFrequency ;
double DisplayError ;

void loop() {
  readBtnState();
  if (encodermov!= 0 ) {
    ActSel+=encodermov;
    encodermov=0;
  }

  GPSTimeDisplay();
  
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(Thermometer);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLUE);
  tft.fillRect( 150,200, 90,24, ST77XX_BLACK);
  tft.setCursor(150,200);
  tft.print(tempC);
  tft.setCursor(200,200);
  tft.print(" C");

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_RED);
  tft.fillRect( 5,200, 12*5,24, ST77XX_BLACK);
  tft.setCursor(5,200);
  tft.print(PPSCount);

  if (FrequencyUpdatePending) {
    UpdateFrequency();
    FrequencyUpdatePending=false;

    if ( Old_2_5_Mhz != Freq_2_5_Mhz ) {
      Si5351_SetFreq(SYNTH_MS_0, Freq_2_5_Mhz ,RDIV_32 );
      Old_2_5_Mhz=Freq_2_5_Mhz;
    }
    
    if ( Old_10_Mhz != Freq_10_Mhz ) {
      Si5351_SetFreq(SYNTH_MS_1, Freq_10_Mhz ,RDIV_8);
      Old_10_Mhz = Freq_10_Mhz;
    }
    
    if ( Old_1_Mhz != Freq_1_Mhz ) {
      Si5351_SetFreq(SYNTH_MS_2, Freq_1_Mhz ,RDIV_64);     
      Old_1_Mhz = Freq_1_Mhz;   
    }
    
    DisplayFrequency =  4 * ( Fcounter - count_error )  / ( timegate - counterdelay ) ;
    DisplayError     = (4 * ( Fcounter - count_error )) % ( timegate - counterdelay ) ;
    DisplayError     = DisplayError / ( timegate - counterdelay ) ;
  
    tft.setTextSize(3); // Frequency
    tft.setTextColor(ST77XX_WHITE);
    tft.fillRect( 5,120, 230,24, ST77XX_BLACK);
    tft.setCursor(5,120);
    tft.print(DisplayFrequency );

    tft.setTextSize(2); // ppm
    tft.setTextColor(ST77XX_GREEN);
    tft.fillRect( 165,126, 65,18, ST77XX_BLACK);
    tft.setCursor(165,126);
    tft.print( DisplayError );
  }
  
  smartDelay(500);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
