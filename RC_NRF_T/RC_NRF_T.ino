#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>

///
/// @mainpage	Robot_32
///
/// @details	PWM for Servos
///
/// @file		Robot_32.ino
/// @brief		Main sketch
///
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
/// @author		Ruedi Heimlicher
/// @date		14.07.2019 20:01
///
/// @copyright	(c) Ruedi Heimlicher, 2019
////// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu

#include "Arduino.h"

#include <ADC.h>
#include <ADC_util.h>

#include <SPI.h>
#include "gpio_MCP23S17.h"
//#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "lcd.h"
#include "analog.h"

#include <EEPROM.h>

#include "nRF24L01.h"
#include "RF24.h"

//#include <RF24Network.h>



LiquidCrystal_I2C lcd(0x27,20,4); 


#define RNF_CE_PIN   10
#define RNF_CSN_PIN   8
#define RNF_IRQ_PIN   9

#define SPI_MOSI  11
#define SPI_MISO 12
#define SPI_CLK 13


#define LX_PIN  20
#define LY_PIN  21
#define RX_PIN  14
#define RY_PIN  15

#define LP_PIN  16
#define RP_PIN  17

#define LT_PIN  7
#define RT_PIN  4


#define OSZIA_PIN                                2

#define OSZIA_LO digitalWrite(OSZIA_PIN,LOW)
#define OSZIA_HI digitalWrite(OSZIA_PIN,HIGH)
#define OSZIA_TOGG digitalWrite(OSZIA_PIN,!(digitalRead(OSZIA_PIN)));

RF24 radio(RNF_CE_PIN, RNF_CSN_PIN);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address

const uint64_t pipe = 0xE8E8F0F0E1LL;


struct PacketData 
{
  byte lxAxisValue;
  byte lyAxisValue;
  byte rxAxisValue;
  byte ryAxisValue;
  byte lPotValue;  
  byte rPotValue;    
  byte switch1Value;
  byte switch2Value;
  byte switch3Value;
  byte switch4Value;  
};
PacketData data;


// Load Wi-Fi library


ADC *adc = new ADC(); // adc object
// Set parameters

#define TEST 1
// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
//#define STARTWERT  2840 // Mitte
#define STARTWERT  2080 // Nullpunkt
#define MAXWERT  4096 // Nullpunkt


#define LOOPLED 3 // 





volatile uint8_t loopstatus = 0;
volatile uint8_t loopcounter = 0;
volatile uint8_t sourcestatus = 1; // local/USB
#define LOCAL  0
#define USB 1
#define FIRSTRUN  1

#define SERIAL_OK 2

byte buffer[64];

byte sendbuffer[64];


elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

volatile uint8_t usbtask = 0;

volatile uint8_t teensytask = 0;

volatile uint16_t aktualcommand = 0;

volatile uint16_t commandarray0[26] = {0};

volatile uint16_t taskarray[8][32] = {0};

volatile uint16_t adressearray[4];
volatile uint16_t eepromadressearray[8][4];
volatile uint16_t speedarray[5];

volatile uint8_t loknummer = 0;

volatile uint8_t speed = 0;



uint8_t minanzeige = 0xFF;
//let GET_U:UInt8 = 0xA2
//let GET_I:UInt8 = 0xB2

// sinus
elapsedMillis sinms;
elapsedMillis sinceblink;


float sinpos = 0;
#define pi 3.14
#define SIN_START   0xE0
#define SIN_STOP   0xE1

#define SPI_CLK   13
#define SPI_MISO  12
#define SPI_MOSI  11
#define SPI_CS    10


#define ANZKANAL       2 // anz Pot


gpio_MCP23S17 mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
uint8_t regA = 0x0;
uint8_t regB = 0;

volatile uint8_t tastencodeA = 0;
volatile uint8_t tastencodeB = 0;
uint8_t tastenstatusA = 0;
//pi.__BEGIN_DECLS

volatile uint8_t tastenadresseA = 0;
volatile uint8_t tastenadresseB = 0;

volatile uint8_t lokaladressearray[ANZKANAL] = {}; // Lok-Adressen
volatile uint8_t lokalcodearray[ANZKANAL] = {}; // Lok-Codes (Richtung, Funktion)

uint8_t tastenstatusB = 0;

uint8_t potarray[ANZKANAL] = {};
uint8_t localpotarray[ANZKANAL] = {};


uint8_t lokalstatus = 0;
elapsedMillis sincelocalrichtung;

//uint8_t potpinarray[4] = {POT_0_PIN,POT_1_PIN,POT_2_PIN,POT_3_PIN};

char* buffercode[4] = {"BUFFER_FAIL","BUFFER_SUCCESS", "BUFFER_FULL", "BUFFER_EMPTY"};

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa

#define HI     0xFEFE  // 1111111011111110
#define LO     0x0202  // 0000001000000010
#define OPEN   0x02FE  // 0000001011111110

#define TIMERINTERVALL  24
#define PAUSE 12
// Utilities
elapsedMillis sinceringbuffer;

elapsedMillis sincewegbuffer;

elapsedMillis sincebatteriespannung;

elapsedMillis sincemcp;

uint16_t abschnittindex = 0; // aktuelles Element in positionsarray

// Create an IntervalTimer object 
IntervalTimer              paketTimer;
volatile uint16_t          timerintervall = TIMERINTERVALL;

IntervalTimer              stromTimer;
volatile uint16_t          batteriespannung = 0;
volatile uint16_t          batteriespannungarray[8] = {0};
volatile uint16_t          batteriespannungmittel = 0;
volatile uint8_t           batteriespannungmittelcounter = 0;
volatile uint8_t           batteriespannungNULL = 330;
volatile uint8_t           pause = PAUSE;
volatile uint8_t           richtung = 1; // vorwaerts

volatile uint8_t           paketpos = 0;
volatile uint8_t           paketmax = 4;

volatile uint8_t           commandpos = 0; // pos im command
volatile uint8_t           bytepos = 0; // pos im Ablauf

uint16_t                   tritarray[] = {LO,OPEN,HI};

int achse0_startwert=0;

//LiquidCrystal_I2C lcd(0x27,20,4); 

void printHex8(uint8_t data) // prints 8-bit data in hex with leading zeroes
{
   Serial.print("0x"); 
  // for (int i=0; i<length; i++) 
   { 
      if (data<0x10) 
      {
         Serial.print("0");
         
      } 
      Serial.print(data,HEX); 
      Serial.println(" "); 
   }
}
// Functions
/*
void OSZI_A_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,LOW);
}

void OSZI_A_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,HIGH);
}

void OSZI_A_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZI_B_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,LOW);
}

void OSZI_B_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,HIGH);
}
*/

void pakettimerfunction() 
{ 
}

void SPI_Init(void)
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   pinMode(SPI_MISO,OUTPUT);
   
 //  SPI_DDR &= ~(1<<SPI_MISO);
 //  SPI_PORT |= (1<<SPI_MISO);
   
   
 //  SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_CLK)|(1<<SPI_SS);
 //  SPI_PORT |= (1<<SPI_SS);
   
   // Enable SPI, Master, set clock rate fck/16 
   SPCR =   (1<<SPE)|
            (1<<MSTR)|
   (1<<SPR0);//|
            //(1<<SPR1);
   
   /*
    Slave init
    // Set MISO output, all others input
    DDR_SPI = (1<<DD_MISO);
    // Enable SPI 
    SPCR = (1<<SPE);
    */
   
}


void LCD_init(void)
{
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   digitalWrite(LCD_RSDS_PIN,1);
   digitalWrite(LCD_ENABLE_PIN,1);
   digitalWrite(LCD_CLOCK_PIN,1);

}

void ADC_init(void) 
{
   batteriespannung=0; // 
   
   adc->adc0->setAveraging(4); // set number of averages 
   adc->adc0->setResolution(10); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  // adc->adc0->enableInterrupts(ADC_0);
   
   
//   delay(100);
   
   
}

void NRF_init(void)
{
   pinMode(RNF_CE_PIN, OUTPUT);
   digitalWrite(RNF_CE_PIN,HIGH);
   pinMode(RNF_CSN_PIN, OUTPUT);
   digitalWrite(RNF_CSN_PIN,HIGH);
   pinMode(RNF_IRQ_PIN, INPUT_PULLUP);
   
}

void stromtimerfunction()
{
   batteriespannung = adc->analogRead(23);
   batteriespannungarray[batteriespannungmittelcounter & 0x07] = batteriespannung;
   batteriespannungmittelcounter++;
   
}


// Add setup code
void setup()
{
   Serial.begin(9600);
 //while (!Serial) ;
   loopstatus |= (1<<FIRSTRUN); // Bit fuer tasks in erster Runde
   delay(100);
 //  analogWriteResolution(16); // 32767
   
   //SPI_Init();
   
   data.lxAxisValue = 127;
   data.lyAxisValue = 127;
   data.rxAxisValue = 127;
   data.ryAxisValue = 127;
   data.lPotValue  = 127;  
   data.rPotValue  = 127;    
   data.switch1Value= 1;
   data.switch2Value = 1;
   data.switch3Value = 1;
   data.switch4Value = 1;  

   uint16_t nrfcounter = 0;
   //NRF_init();
   
   // Define the radio communication
   /*
   if (!radio.begin()) 
   {
       while (nrfcounter < 1000) {
          nrfcounter++;
       }  // hold in infinite loop
      Serial.println(F("radio hardware is not responding!!"));
   }
   else
   {
      Serial.println(F("radio hardware is OK"));
      radio.openWritingPipe(address);
      radio.setAutoAck(false);
      radio.setDataRate(RF24_250KBPS);
      radio.setPALevel(RF24_PA_LOW);
   }
   */
   radio.begin();
   if ( radio.available() )
   {
      Serial.println("radio OK");
   }
   else
   {
      Serial.println("No radio available");
   }
   
   paketTimer.begin(pakettimerfunction,timerintervall);
   paketTimer.priority(0);
   
   stromTimer.begin(stromtimerfunction, 5000);
   
   pinMode(LOOPLED, OUTPUT);
   
   // FTM0   Pins: 5, 6, 9, 10, 20, 21, 22, 23
   // FTM1   3, 4   
   // FTM2   25, 32
   analogWriteFrequency(5, 50);
   Serial.println(F("RawHID RC_NRF"));
 
    
   LCD_init();
   
   
   mcp0.begin();
   /*
    • PortA registeraddresses range from 00h–0Ah
    • PortB registeraddresses range from 10h–1Ah
    PortA output, PortB input: Direction 1 output: direction 0
    0x0F: A: out B: in
    */

   //mcp0.gpioPinMode(0x00FF); // A Ausgang, B Eingang
   mcp0.gpioPinMode(0xFFFF); // alle input
   
   //mcp0.portPullup(0x00FF); 
   mcp0.portPullup(0xFFFF);// alle HI
   
   mcp0.gpioPort(0xCC33);

   
   EEPROM.begin();
    
   delay(100);
   usbtask = 0;
   adressearray[0] = LO;
   adressearray[1] = HI;
   adressearray[2] = LO;
   adressearray[3] = OPEN;
   
   speed = 0;
   Serial.print("speed: ");
   Serial.print(speed);
   Serial.print("\n");
   Serial.print("a: ");
   Serial.print(speed & (1<<0));
   Serial.print(" b: ");
   Serial.print(speed & (1<<1));
   Serial.print(" c: ");
   Serial.print(speed & (1<<2));
   Serial.print(" d: ");
   Serial.print(speed & (1<<3));
   Serial.print("\n");

  // init_analog();
   for (uint8_t i=0;i<4;i++)
   {
      Serial.print(" i: "); Serial.print(i);
      Serial.print(" data: ");Serial.print(speed & (1<<i));
      Serial.print("\n");
      if (speed & (1<<i))
      {
         Serial.print("HI");
         speedarray[i] = HI; 
      }
      else
      {
         Serial.print("LO");
         speedarray[i] = LO; 
      }
      Serial.print("\n");
   }
 
   aktualcommand = OPEN;
   
   
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(100);
   lcd_puts("Teensy");

   
   ADC_init();
   delay(100);
   Serial.print("setup: ");
   lcd.init();
   lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("RC_NRF");
    _delay_ms(200);
    
 }

// Add loop code
void loop()
{
#pragma mark mcp
   if (sincemcp > 10)
   {
      sincemcp = 0;
         
      // Pot auslesen
      data.lxAxisValue = map(adc->analogRead(LX_PIN),0, 1023, 0, 255);
      data.lyAxisValue = adc->analogRead(LY_PIN);
      data.rxAxisValue = adc->analogRead(RX_PIN);
      data.ryAxisValue = adc->analogRead(RY_PIN);
      
      
      for (uint8_t i=0;i<ANZKANAL;i++)
      {
         //localpotarray[i] = adc->analogRead(potpinarray[i]); // 8 bit
 //        sendbuffer[16+i] = localpotarray[i];
      }
      // Send the whole data from the structure to the receiver
      
       radio.write(&data, sizeof(PacketData));
   }
#pragma mark batteriespannung 
   if (sincebatteriespannung > 200)
   {
      sincebatteriespannung = 0;
      //     batteriespannung = adc->analogRead(CURR_PIN); // in stromtimerfunktion
      
      //Serial.print(" data: \t");
      batteriespannungmittel = 0;
      for (uint8_t i=0;i<8;i++)
      {
         
         //       Serial.print(batteriespannungarray[i]);
         //       Serial.print("\t");
         batteriespannungmittel += batteriespannungarray[i];
      }
      
      //    Serial.print("\t");
      batteriespannungmittel /= 8;
 //     Serial.print("batteriespannungmittel: ");
 //     Serial.print(batteriespannungmittel);
      // Serial.print(byte(0));
      
  //     Serial.print("\n");
      
      //  lcd.setCursor(4,0);
      if (batteriespannung < 0xFF)
      {
         
         uint8_t anzeige = (batteriespannungmittel);
         anzeige = anzeige ^ 0xFF;
         if (anzeige < minanzeige)
         {
            minanzeige = anzeige;
         }
         lcd_gotoxy(0, 3);
         
         lcd_putint(0xFF - batteriespannungmittel);
         lcd_putc(' ');
         lcd_putint(batteriespannungmittel);
         lcd_putc(' ');
      }
      else 
      {
         //  lcd.print("         ");
      }
      ////  lcd.setCursor(18,0);
      uint8_t pos = (batteriespannungmittel)/10;
       
      ////  lcd.print("*");
      sendbuffer[10] = 0xAB;
      sendbuffer[12] = batteriespannung & 0x00FF;
      sendbuffer[13] = (batteriespannung & 0xFF00)>>8;
   
   }
#pragma mark blink 
   if (sinceblink > 1000)
   {
      sinceblink = 0;
      loopcounter++;
      //_delay_ms(10);
      //pinMode(LOOPLED, OUTPUT);
      digitalWriteFast(LOOPLED, !digitalReadFast(LOOPLED));
       lcd_gotoxy(16, 0);
      lcd_puthex(sourcestatus);
       
       
   }
   
   #pragma mark USB
   int n;
   n = RawHID.recv(buffer, 10); // 
   if (n > 0) 
   {
      // the computer sent a message.  Display the bits
      // of the first byte on pin 0 to 7.  Ignore the
      // other 63 bytes!
      //Serial.print(F("Received packet, erstes byte: "));
      //Serial.println((int)buffer[0]);
      //     for (int i=0; i<8; i++) 
      {
         //       int b = buffer[0] & (1 << i);
         //       Serial.print((int)buffer[i]);
         //       Serial.print("\t");
         //digitalWrite(i, b);
      }
      //     Serial.println();
      //     Serial.print(hb);
      //     Serial.print("\t");
      //     Serial.print(lb);
      //     Serial.println();
      
      // ************************************
      usbtask = buffer[0]; // Auswahl 
      // ************************************
      
      Serial.println("usbtask ");
      Serial.print("******************  usbtask *** ");
      printHex8(usbtask);
      for (int i=0; i<24; i++) 
      {
         Serial.print(buffer[i]);
         Serial.print(" ");
      }
      Serial.print("\n");
      
  #pragma mark TASK 
      if (sourcestatus & 0x02)
      {
         switch (usbtask)
         {
            case 0xA0: // address
            {
               
                 
            }break;
              
         }// switch usbtask
         
      } // if localstatus & 0x02
      Serial.println("USB END");
   } // n>0
   
#pragma mark local
   else if (sourcestatus & 0x01)
   {
     // if (digitalReadFast(SOURCECONTROL) == 1)
      
   } // local
   

#pragma mark sincewegbuffer 
   
   if ((sincewegbuffer > 1000))// && (usbtask == SET_WEG)) // naechster Schritt
   {
      sendbuffer[10] = 0xAB;
      sendbuffer[12] = batteriespannung;
      
      n = RawHID.send(sendbuffer, 100);
      if (n > 0) 
      {
         //        Serial.print(F("Transmit packet "));
         //        Serial.println(packetCount );
         packetCount = packetCount + 1;
      } else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
         
   }   

#pragma mark sinceringbuffer    
   if ((sinceringbuffer > 32))// && (usbtask == SET_RING)) // naechster Schritt
   {
      sinceringbuffer = 0;
   }
   
   // every 4 seconds, send a packet to the computer
   if (msUntilNextSend > 4000) 
   {
      msUntilNextSend = msUntilNextSend - 2000;
      
 
   }
} // loop
