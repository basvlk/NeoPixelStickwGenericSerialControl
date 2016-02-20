
/**
  This is the contolr program for a stck of 8 NeoPixels, by Bas Vellekoop. I'm migrating the serial communication code from leds_for_SalonV3, bear with me
  A big part of the program is about Serial communication
  It uses a lot of space for Global Variables - something I'll need to figure out. For now the solution is to usa an Arduino Mega

  TO DO:
  1. Change that control program doesn't send 3 bytes (RGB) but just one colorByte
  2. Look at global / local variables and memory usage

 **************************
 ***** SERIAL COMMUNICATION
 **************************
   The main objectives are:
   a fast way of receiving short commands, responding with low latency
   also allowing longer messages to be sent, where latency is not a problem
   Robust for messages being too short or too long, or invalid
   ability to send error messages back
   a Diagnostic on/off function, sending a lot more information back

   Principle:
   (1) Header Bytes and start reading from Serial
   - the first Byte of any message is '255'
   - the second byte is 'Mode' which is used further down in the program for 'cases'
   - the third Byte is 'DataLength" and says how many bytes will follow: this is the actual data. if '0' it means no further Bytes follow
   - As a result, messages are always a minimum of THREE Bytes: 1:Begin ("255") 2:Mode 3:DataLength
   - the program waits with reading in Bytes until there are a minimum of 3 Bytes in the buffer. Then it starts reading them.
   - Once >3 Bytes are received if the first is a '255' it kicks off a reading cycle. If a different value is received, it is discarded
   (2) Reading in the DataBytes
   - if DataLength > 0, first a check is being done on whether the number of Bytes in the buffer is correct for the Datalength of the message:
   -- If there are less Bytes in the buffer than expected, the program waits until (CommsTimeout) and if they haven't arrived, discards the Bytes
   -- If there are too many Bytes in the buffer, the program doesn't trust the information, and discards all Bytes in the buffer.
   - If the Bytes in the buffer correspond to the Datalength, the bytes can be read either into an array or into variables - but this is done within the 'cases' or Modes
   - Either way, once the program continues into a Mode, the amount of data in the serial buffer is valid, it's not too short and not too long, no further checking is required.
   (3) 2 kinds of Modes:
   0-99 are "OnceModes": they make the Arduino 'do' something once, like:
       (a) Do things like switch all LEDs on and off (1=Off, 2=White, 3=Red. 4=Green, 5=Blue
       (b) Mode 10-98 is to activate presets 10-98
       (c) Mode 9 sets the variable preset StateX, based on incoming bytes
       (d) Mode 99 is for changing the Diagnostic settings
   100-199  Continuous modes (ContMode): Modes that have time-based effects and need to be visited every loop. until the mode is changed, we want the program to keep executing this mode.


  PLANNED CHANGES:
  Problem: The current version stops the program from running until 'DataLength' bytes have arrived. If they aren't already all in the buffer, the program waits for all of them to arrive before continuing.
  ==> Planned change: add a stopbyte to the serial comms so the whole program can keep running while (slow) serial data is being read in. The read section will then just read data into al buffer until


 **/

//LED SETUP
#include <Adafruit_NeoPixel.h>
#define PIN 12
const  int nLEDs = 8; // Number of RGB LEDs:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDs, PIN, NEO_GRB + NEO_KHZ800);

//PROGRAM CONTROL
const byte ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
unsigned long previousMillis = 0;  // will store last time at the end of the previous loop
unsigned long currentMillis = 0;  // will store the current time
unsigned long CommsTimeout = 200;    // When the program is expecting X bytes to arrive, this is how long it will wait before discarding

/// For Cont loop timing
unsigned long ContLoopMillis = 0;  // will store last time at the end of the previous CONT loop
byte LoopDelayCounter = 0; // allows to skip a number of loopcycles in Cont Modes.
unsigned long ContCurrentStep = 0; //For iterations in Cont Loops, to keep the 'current step' and add +1 each time the loop runs
byte invert = 1; // allows for fade in to turn to fadeout
byte ContLoopIteration = 0;

//DIAGNOSTIC TOOLS
byte Diagnostic = 1;                // switches on all kinds of diagnostic feedback from various locations in the program
byte LooptimeDiag = 1;              // minimal feedback for checking efficiency: only feeds back looptime
int ArrayDiag = 1;                 // if switched on, prints all arrays every cycle
unsigned long Slowdown = 500;                  // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
unsigned long msTable[10] = {0, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000}; //Delay values in ms to 'Slow Down' program for diagnostic purposes
byte LoopIteration = 0;             // to track loop iterations

// SERIAL- required for the core functionality of the Serial communication
byte UpdateBytesInBufferToSerial = 1;                // siwtch to only write to serial if BytesInBuffer has changed
byte BytesInBuffer = 0;             // number of unread Bytes waiting in the serial buffer
byte LastReceivedData[64];
byte DiscardedBytes = 0;            // number of Bytes discarded (read out of the serial buffer but not used) since the last start of a read operation. Indicates something is wrong
byte Mode = 0;
byte ContMode = 0;
byte OnceMode = 0;
int DataLength = 0;
byte ReadInBuffer[64] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int BytesRead = 0;

// PRESETS
int STATE10[nLEDs * 3] = {100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0}; // 8 x 3 (8x RGB)
int STATE11[nLEDs * 3] = {100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0}; //
int STATE12[nLEDs * 3] = {0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0}; //
int STATE13[nLEDs * 3] = {0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100}; //
int STATE14[nLEDs * 3] = {100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100,}; //

void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  pinMode(ArduinoLedPin, OUTPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  Serial.println("Hello.");
  LoopIteration = 0;
}


//**********************************************************
//  *************         M A I N       **********************
//**********************************************************

void loop()
{
  // Start of loop housekeeping
  currentMillis = millis();
  ++LoopIteration;
  LoopBlink(LoopIteration);

  // delay(1); /////REMOVE THIS LINE WHEN MORE CODE ADDED!!: somehow the serial communication deosn't work well if there is no slight delay

  if (Diagnostic == 1) {                  //Diag
    delay(Slowdown);                      //Diag
    Serial.print(F("[ **** NEW LOOP: "));    //Diag
    Serial.println(LoopIteration);        //Diag
    Serial.print(F("[ currentMillis: "));    //Diag
    Serial.println(currentMillis);        //Diag
    Serial.print(F("[ Slowdown: "));         //Diag
    Serial.println(Slowdown);             //Diag
  }
  // End of loop housekeeping

  //*************       C H E C K   A V A I L A B L E  B Y T E S  S E C T I O N   *****************
  UpdateBytesInBufferToSerial = !((BytesInBuffer == Serial.available() ));
  BytesInBuffer = Serial.available();
  if (UpdateBytesInBufferToSerial) {
    Serial.print("]");
    Serial.println(BytesInBuffer);
  } //Only update BytesInBuffer value to Serial port if there has been a change in the value

  if (BytesInBuffer == 0) {
    DiscardedBytes = 0;
  }  // if data does not start with '255', it is invalid, discarded, and the next loop looks again for '255'. 'DiscardedBytes' keeps track of how many loops have thrown away 1 Byte. once there is nothing left in the buffer it is reset, so the next is counted separately

  // Start reading in data once 3 Bytes of data have arrived
  if (BytesInBuffer > 2)
  {
    if (Serial.read() != 255) // IF 3 bytes or more AND the first is NOT '255'; cycle back to start without doing anything with the read Byte:
      // effectively this just removes the first Byte, and leaves the remaining in the buffer, so the next one can be evaluated in the next main loop execution
    {
      ++DiscardedBytes;
      Serial.print("[ ERROR: Bytes received not starting with '255' Discarded: ");
      Serial.println(DiscardedBytes);
    } //End invalid data section (ie data did not start with '255' and is non-255 byte is discarded. The rest is left intact in case it is the start of a valid message

    else //************* START VALID READING (>3 & 255 startByte)  *****************************************************
    {
      // SECTION 1: MODE and LENGTH
      if (Diagnostic == 1) {                    //Diag
        Serial.println(F("[ Entered reading section, ergo >=3 Bytes in buffer, first is 255"));
      }                                        //Diag
      byte TempMode = Mode;//If the read data turns out to be valid, we'll make  Mode=TempMode. let's see first if it passes the tests

      TempMode = Serial.read();
      DataLength = int( Serial.read());
      BytesInBuffer = Serial.available();
      BytesRead = 0;

      if (Diagnostic == 1) {                     //Diag
        Serial.print(F("[ Received Mode Byte: "));             //Diag
        Serial.print(TempMode);                      //Diag
        Serial.print(F(" // Current ContMode: "));   //Diag
        Serial.print(ContMode);                  //Diag
        Serial.print(F(" // Current OnceMode: "));   //Diag
        Serial.println(OnceMode);                //Diag
        Serial.print(F("[ DataLength: "));       //Diag
        Serial.print(DataLength);                //Diag
        Serial.print(F(" - Bytes remaining in Buffer: ")); //Diag
        Serial.println(BytesInBuffer);           //Diag
      }                                          //Diag

      // SECTION 2: Read Serial data into ReadInBuffer
      BytesRead = int(Serial.readBytes( ReadInBuffer, 64));
      BytesInBuffer = Serial.available();
      if (Diagnostic == 1) {                     //Diag
        Serial.print(F("Bytes Read = "));
        Serial.print(BytesRead);
        Serial.print(F(" // Bytes still available at Serial = "));
        Serial.println(BytesInBuffer);
      }
      // If data is as expected:
      if (BytesRead == DataLength) {
        Mode = TempMode; // valid data, Mode can be set
        SetMode(Mode);
        if (Diagnostic == 1) {                     //Diag
          Serial.print(F("[ expected amount of bytes were read into buffer "));             //Diag
        }
      }
      else {//SECTION 3: if things aren't quite right
        // Serial buffer not emptied - too MANY BYTES
        if (BytesInBuffer != 0) {
          Serial.print(F("[ ERROR: "));
          Serial.print(BytesInBuffer);
          Serial.println(F(" Bytes still in buffer. Dumping all remaining data in Serial buffer"));
          char dump[BytesInBuffer];
          Serial.readBytes(dump, BytesInBuffer);
        }
        // More Bytes read than expected - too MANY BYTES
        if (BytesRead > DataLength) {
          Serial.print(F("[ ERROR: "));
          Serial.print(BytesRead, DEC);
          Serial.println(F(" Bytes were read, but expected less (Datalength): Datalength = "));
          Serial.print(DataLength);
        }
        //NOT ENOUGH BYTES
        if (BytesRead < DataLength) {
          Serial.print(F("[ ERROR: Reading timed out before sufficient Bytes received "));
          Serial.print(F("  Bytes Read = "));
          Serial.print(BytesRead);
          Serial.println (F(" // Bytes expected (DataLength)= "));
          Serial.print(DataLength);
        }
      }
      // End of valid reading section

    }

  } // End reading / discarding data section which only runs when there are 3 Bytes or ore in the buffer


  ////// BRIDGE section, between validating the incoming data, and reading in the data (if valid)
  if (Diagnostic == 1) {                    //Diag
    Serial.print("[ Last set Mode: ");     //Diag
    Serial.print(Mode);                     //Diag
    Serial.print(" // Current ContMode: "); //Diag
    Serial.print(ContMode);                 //Diag
    Serial.print(" // Current OnceMode: "); //Diag
    Serial.println(OnceMode);               //Diag
  }




  //*************       C O N T I N U O U S  M O D E S      **********************
  // this section represents modes that run continuously, once with every main loop cycle- to be used for time-base effects

  switch (ContMode)

  { //Start MODES Section
    case 100:
      {
        if (Diagnostic == 1) {
          Serial.println("[ Continuous Mode 0 - not doing anything");
        }
        break;
      }
    case 101:
      {
        if (Diagnostic == 1) {
          Serial.println("[ Continuous Mode 101 - One blinking green LED with 500ms delay");
        }
        //       simple LED blinkcycle
        int colorByte = 0;
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        }
        strip.show();              // Refresh LED states

        delay(500);

        int pix = 0 ;
        int r = 5;
        int g = 10;
        int b = 5 ;
        colorByte = strip.Color(r, g, b);
        strip.setPixelColor(pix, strip.Color(r, g, b)); // Set new pixel 'on'
        strip.show();              // Refresh LED states
        delay(500);
        break;
      }

    case 102:
      {
        if (Diagnostic == 1) {
          Serial.println("[ Continuous Mode 102 - full strip blink");
        }

        //       simple LED blinkcycle
        int colorByte = 0;
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        }
        strip.show();              // Refresh LED states

        delay(500);

        int r = 20;
        int g = 0;
        int b = 0 ;
        colorByte = strip.Color(r, g, b);
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(6, colorByte); // set new colors, but don't refresh!
        }
        strip.show();              // Refresh LED states
        delay(500);

        break;
      }
    case 103:
      {
        if (Diagnostic == 1) {
          Serial.println("[ Continuous Mode 103 - all blinking red with 500ms delay");
        }
        //       simple LED blinkcycle
        int colorByte = 0;
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        }
        strip.show();              // Refresh LED states

        delay(500);

        int pix = 3 ;
        int r = 5;
        int g = 0;
        int b = 0 ;
        colorByte = strip.Color(r, g, b);
        for (int pix = 0; pix < strip.numPixels(); pix++) {
          strip.setPixelColor(pix, strip.Color(r, g, b)); // Set new pixel 'on'
        }
        strip.show();              // Refresh LED states
        delay(500);
        break;
      }

  }

  //*************       L A S T  P A R T  O F  M A I N  P R O G R A M      **********************
  if (ArrayDiag == 1)
  {
    Serial.println("arrays have been commented out in this UNO version");
    /*
      ArrayToSerial(STATEX, nLEDs * 3);
      ArrayToSerial(STATE10, nLEDs * 3);
      ArrayToSerial(STATE11, nLEDs * 3);
      ArrayToSerial(STATE12, nLEDs * 3);
      ArrayToSerial(STATE13, nLEDs * 3);
      ArrayToSerial(STATE14, nLEDs * 3);
      ArrayToSerial(STATE15, nLEDs * 3);
    */
  }
  if (Diagnostic == 1) {
    Serial.print("[ **** END OF LOOP ");
    Serial.println(LoopIteration);
  }

  currentMillis = millis();
  if (LooptimeDiag == 1) {
    Serial.print("[ Looptime: ");
    Serial.println(currentMillis - previousMillis);
  }
  previousMillis = currentMillis;


} //End main loop



// OLD STUFF FROM NEOPIXELS FOR SALON

/**


  case 2: //All LED
  {
  if (Diagnostic)
    { //Diag start
     Serial.println("[ Mode: All LEDs");
    } //Diag end

  int r = Instruction[1];
  int g = Instruction[2];
  int b = Instruction[3];
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b) ); // Erase pixel, but don't refresh!
  }
  strip.show();              // Refresh LED states
  break;
  }
  case 3: //color chase
  {
  if (Diagnostic)
    { //Diag start
     Serial.println("[ Mode: Chase");
    } //Diag end

  int r = Instruction[1];
  int g = Instruction[2];
  int b = Instruction[3];
  int time = (2 * Instruction[0]) ;
  colorChase(strip.Color(r, g, b), time);
  break;
  }
  case 4: //color wipe
  {
  if (Diagnostic)
    { //Diag start
     Serial.println("[ Mode: Wipe");
    } //Diag end

  int r = Instruction[1];
  int g = Instruction[2];
  int b = Instruction[3];
  int time = (2 * Instruction[0]) ;

  colorWipe(strip.Color(r, g, b), time);
  break;
  }

  default:
  {
  Serial.println("[ No Known mode engaged");
  break;
  };
  } //end MODES section

  }// end Main section




  // Fill the dots progressively along the strip.
  void colorWipe(uint32_t c, uint8_t wait) {
  int i;

  for (i=0; i < strip.numPixels(); i++) {
  strip.setPixelColor(i, c);
  strip.show();
  delay(wait);
  }
  }

  // Chase one dot down the full strip.
  void colorChase(uint32_t c, uint8_t wait) {
  int i;

  // Start by turning all pixels off:
  for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);

  // Then display one pixel at a time:
  for(i=0; i<strip.numPixels(); i++) {
  strip.setPixelColor(i, c); // Set new pixel 'on'
  strip.show();              // Refresh LED states
  strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
  delay(wait);
  }
  delay(500);
  strip.show(); // Refresh to turn off last pixel
  }
  }
**/


//*****************************************************************
//*************       F U N C T I O N S      **********************
//*****************************************************************
// *** PROGRAM RUNNING FUNCTIONS

// Blink ArduinoLED to show program is running. Toggles On/Off every loop
void LoopBlink(int Loop)
{
  if (Loop % 2)
  {
    digitalWrite(ArduinoLedPin, HIGH);
  }
  else
  {
    digitalWrite(ArduinoLedPin, LOW);
  }
}// End blinkLed

// PrintArrayToSerial: A diagnostic printing out full arrays to the serial port
void ArrayToSerial(byte Array[], int N) {
  for (int i = 0; i < N ; i++)
  {
    Serial.print(" ");
    Serial.print(Array[i], DEC);
    Serial.print(" ");
  }
  Serial.println("");
} // END PrintArrayToSerial

void SetDiagnostic() //Mode 99 is CONFIG. Bytes set: Diagnostic, Delay
{
  Serial.println("[ Entering Diagnostic change");
  Diagnostic = Serial.read();
  int i = Serial.read();
  if (i < 10) {
    Slowdown = msTable[i];
  }
  else
  { Serial.print("[ ERROR: Slowdown value > 9");
  }
  LooptimeDiag = Serial.read();
  ArrayDiag = Serial.read();
  CommsTimeout = msTable[Serial.read()];
  Serial.setTimeout(CommsTimeout);
  Serial.print("[ Diagnostic set to: ");
  Serial.println(Diagnostic);
  Serial.print("[ Slowdown set to: ");
  Serial.println(Slowdown);
  Serial.print("[ CommsTimeout set to: ");
  Serial.println(CommsTimeout);

} // END SetDiagnostic


void SetMode(byte inputmode)
{
  if (inputmode > 99) {
    ContMode = inputmode;
  }
  if (inputmode < 99) {
    OnceMode = inputmode;
    ContMode = 0;
  }
  if (inputmode == 99) {
    SetDiagnostic();
  }
} //END SetMode


/*
  // ********** LED SETTING AND EFFECT FUNCTIONS*******

  // Array to NeoPixels
  void ArrayToPixels(byte Array[], int N) {
  for (int i = 0; i < N / 3; i++)
  {
    int pix = i;
    int r = Array[i * 3];
    int g = Array[i * 3 + 1];
    int b = Array[i * 3 + 2];
    strip.setPixelColor(pix, strip.Color(r, g, b)); // Set new pixel 'on'
    strip.show();              // Refresh LED states
  }
  }


  // Fill the dots one after the other with a color
  void colorWipe(uint32_t c, uint8_t wait) {
  if (!LoopDelayCounter) {
    ContCurrentStep +=1;
   if (ContCurrentStep = strip.numPixels) {ContCurrentStep = 0;}
      strip.setPixelColor(ContCurrentStep, c);
      strip.show();
    }

  LoopDelayCounter++ ;
  if (LoopDelayCounter > wait) {
    LoopDelayCounter = 0;
  }
  }


  //
  void rainbow(uint8_t wait) {
  uint16_t i, j;


  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);

  }
  }

  /// attempting better timing:
  void newrainbow(uint8_t wait) {
  uint16_t i;
  ContCurrentStep += 1;
  if (ContCurrentStep == 256) {
    ContCurrentStep = 0;
  }
  // Serial.println(ContCurrentStep);

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel((i + ContCurrentStep) & 255));
  }
  strip.show();
  delay(wait);
  }


  // Slightly different, this makes the rainbow equally distributed throughout
  void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
  }


  // trying with corrected timgin
  void newrainbowCycle(uint8_t wait) {
  uint16_t i, j;

  ContCurrentStep += 1;
  if (ContCurrentStep == 256 * 5) {
    ContCurrentStep = 0;
  }

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + ContCurrentStep) & 255));
  }
  strip.show();
  delay(wait);

  }


  //Theatre-style crawling lights.
  void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0); //turn every third pixel off
      }
    }
  }
  }

  //Theatre-style crawling lights with rainbow effect
  void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) { // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0); //turn every third pixel off
      }
    }
  }
  }


  // COLORWHEEL input from 0 to 255 and color goes R-Rb-rb-rB-Bg-bg-bG-G-rG-rg-Rg-R:
  // Input a value 0 to 255 to get a color value.
  // The colours are a transition r - g - b - back to r.
  uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  }

  void blinker(uint8_t cycletime) {
  //float Fraction = 1;
  // Fraction = 255*((millis() - PreviousMillis) / cycletime);
  //   stepsize = (int) Fraction;
  // previousMillis

  if ((millis() - ContLoopMillis) > cycletime)
  { LoopDelayCounter++ ;
    if (LoopDelayCounter % 2) {
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(255, 255, 255)); // Erase pixel, but don't refresh!
      }
    }
    else
    {
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
      }
    }

    strip.show();
    ContLoopMillis = millis();
  }
  }


  // FADER
  void fader(uint32_t cycletime) {

  int stepsize = constrain ( map ( (millis() - ContLoopMillis), 0, cycletime , 0, 255) , 0, 255);
  if (stepsize) {
    ContLoopMillis = millis(); // if the stepsize = 0 (ie the change is superslow and LEDs don't need to be updated
  }
  ContCurrentStep +=  stepsize;
  if (ContCurrentStep > 255) {
    ContCurrentStep = 0;
    //    Serial.println("  OVER 255 WHOOHOO ");
    //  invert = -invert;
  }
  Serial.print("  cycletime= ");
  Serial.print(cycletime);
  Serial.print("  (millis() - ContLoopMillis)= ");
  Serial.print((millis() - ContLoopMillis));

  Serial.print("stepsize=");
  Serial.print(stepsize);
  Serial.print("  ContCurrentStep= ");
  Serial.print(ContCurrentStep);
  Serial.print("  invert= ");
  Serial.println(invert);


  {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(ContCurrentStep, 0, 0)); // Erase pixel, but don't refresh!
    }
    strip.show();
  }
  if (stepsize) {
    ContLoopMillis = millis(); // if the stepsize = 0 (ie the change is superslow and LEDs don't need to be updated every cycle, keep counting time
  }
  }

*/






















