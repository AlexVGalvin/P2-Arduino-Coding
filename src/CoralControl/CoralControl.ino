// A basic everyday NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <math.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 13
#define MOTOR_PIN 7
#define UV_PIN 8

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 8

#define brightness 10

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


// setup() function -- runs once at startup --------------------------------

int highTemp = 27;  //Temperature where lights will depict coral that is starting to die
#define tempRange 5

float exponent = log(255)/tempRange;

  int red[4] = {255, 127, 0, 127};
  int green[4] = {0, 0, 255, 255};
  int blue[4] = {0, 255, 255, 0};

void setup() {
  pinMode(UV_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
  Serial.begin(9600);
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(brightness); // Set BRIGHTNESS (max = 255)
}


// loop() function -- runs repeatedly as long as board is on ---------------
float ourTemperature = 27;
float otherTemperature = 0; //This is set to zero to work with the if statement below
float temperature = 27;
void loop() {
  ourTemperature = getTemp();
  if(Serial.available())
  {
    otherTemperature = Serial.parseFloat();
    Serial.println(otherTemperature);
    while(Serial.available()) {
      Serial.read();
    } 
  }
  if (otherTemperature != 0) // // Aquarium temperature is 67% contolled by local temperature If we don't give it a temperature, then it is 100% our measured temperature
    temperature = (2*ourTemperature + 1*otherTemperature) / 3;
  else
    temperature = getTemp(); //Gets the temperature from the waterproof temperature sensor
  
  //Print out the temperature
  Serial.print(temperature);
  Serial.println(" deg C");

  // Wait 100ms before reading voltage again
  delay(100);
  //Temperature Control Ends

  if (temperature > highTemp+tempRange) //When the temperature gets too high, the motor turns off and the UV LEDs turn off
  {
    digitalWrite(MOTOR_PIN, HIGH);  //The motor pin is set to high since the motor is connected to a transistor which allows the motor to be controlled
                                    //by the Arduino, while being powered by the external power supply. Setting it to high stops current in the transistor
    digitalWrite(UV_PIN, LOW);      
    updateColors(0);                //Updates the colors on the Neo Pixels. The 0 is a delay between updates which controls the speed of the color changes,
                                    //Which can be 0 since the lights are just turned off when the temperature is too high.
  }
  else                              //If the temperature is low enough, the Motor runs, UV LEDs turn on, and the corals get a rainbow effect.
  {
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(UV_PIN, HIGH);
    updateColors(15);
  }
}

int clamp(int x, int maximum, int minimum)
{
  if (x > maximum)
    x = maximum;
  else if (x < minimum)
    x = minimum;
  return x;
}

unsigned long CurrentTime = millis();     //Makes it only check the temperature every 2 seconds.
unsigned long PreviousTime = CurrentTime;

// Assigns a color tetrad (i.e. two pairs of complimentary colors) 
// to the first four NeoPixels in the strip
void updateColors(int wait) {
  uint32_t color = 0;
  uint8_t saturation = 0;
  uint8_t value = 255;
  uint16_t hue = 0;


  for (int t = 0; t<255; t++) {
    // Temperature is 2/3rds influenced locally, 1/3 by the other tank (assuming other temperature is given)
    CurrentTime = millis();
    if (CurrentTime >= (PreviousTime + 250))
    {
      updateTemperature();
    }
    
    if (temperature > highTemp + tempRange) {
      saturation = 0;
    } else if (temperature > highTemp) {
      saturation = (255/tempRange) * ((highTemp + tempRange) - temperature);
    } else {
      saturation = 255;
    }
    
    for (int i = 0; i < 4; i++){
     // Move one quarter of the color wheel per pixel
     // Hue solely depends on a pixel's position
     // Since hue is a uint, it will wrap around when it goes over 65535
     hue = hue + 16384;
    
      color = strip.ColorHSV(hue, saturation, saturation);

      // in HSV, S == saturation, 0 < S < 255
      // saturation dermines a *hue's* intensity. 
      // i.e. if s=255, the hue is intense. if S=10, the color is off-white "tinged" with the hue
      // V == value, 0 < V < 255
      // value determines the *overall* intensity. 
      // if V=0, the color is black. if V=255, the color is very bright.

      if (i == 0) //Setting a color to each coral
      {
        strip.setPixelColor(i, color);  
        strip.setPixelColor(i+3, color); 

      }
      else if (i == 1)
      {
        strip.setPixelColor(i, color);
        strip.setPixelColor(i+1, color); 
      }
      else if (i == 2)
      {
        strip.setPixelColor(i+2, color);
        strip.setPixelColor(i+5, color); 
      }
      else if (i == 3)
      {
        strip.setPixelColor(i+2, color);
        strip.setPixelColor(i+3, color); 
      }

  }
  strip.show();
  hue += 256;
  delay(wait);
  }
}

void updateTemperature() {
  ourTemperature = getTemp();
  CurrentTime = millis();
  PreviousTime = CurrentTime;
  if (otherTemperature != 0) {
    temperature = (2*ourTemperature + 1*otherTemperature) / 3;
  } else {
    //If we don't give it a temperature, then it is 100% our measured temperature
    temperature = ourTemperature;
  }
  Serial.print(temperature);
  Serial.println(" deg C");
}


void customRainbow(int wait)
{
  for (int j = 0; j <= 255; j++)
  {
  for (int i = 0; i < 4; i++)
  {
    if (blue[i]==255)
    {
      if (green[i]!=0)
        green[i]-=10;
      else
        red[i]+=10;
    }
    else if (green[i]==255)
    {
      if (red[i]!=0)
        red[i]-=10;
      else
        blue[i]+=10;
    }
    else if (red[i]==255)
    {
      if (blue[i]!=0)
        blue[i]-=10;
      else
      green[i]+=10;
    }
    red[i] = clamp(red[i], 255, 0);
    green[i] = clamp(green[i], 255, 0);
    blue[i] = clamp(blue[i], 255, 0);

    if ((blue[i]==255) && (red[i]==255))
    {
      blue[i]-=20;
    }

    CurrentTime = millis();
    if (CurrentTime >= (PreviousTime + 2000))
    {
      ourTemperature = getTemp();
      Serial.print(temperature);
      Serial.println(" deg C");
      CurrentTime = millis();
      PreviousTime = CurrentTime;
      if (otherTemperature != 0) //If we don't give it a temperature, then it is 100% our measured temperature
        temperature = (2*ourTemperature + 1*otherTemperature) / 3;
      else
        temperature = ourTemperature;
    }

    int colorChange = 0;
    if (temperature > highTemp)
    {
      colorChange = exp(exponent*(temperature-highTemp))-1;
    }

    int changedRed = red[i]+colorChange;
    int changedGreen = green[i]+colorChange;
    int changedBlue = blue[i]+colorChange;

    changedRed = clamp(changedRed, 255, 0);
    changedGreen = clamp(changedGreen, 255, 0);
    changedBlue = clamp(changedBlue, 255, 0);
    
   
    if (i == 0) //Setting a color to each coral
    {
      strip.setPixelColor(i, changedRed, changedGreen, changedBlue);
      strip.setPixelColor(i+3, changedRed, changedGreen, changedBlue); 

    }
    else if (i == 1)
    {
      strip.setPixelColor(i, changedRed, changedGreen, changedBlue);
      strip.setPixelColor(i+1, changedRed, changedGreen, changedBlue); 
    }
    else if (i == 2)
    {
      strip.setPixelColor(i+2, changedRed, changedGreen, changedBlue);
      strip.setPixelColor(i+5, changedRed, changedGreen, changedBlue); 
    }
    else if (i == 3)
    {
      strip.setPixelColor(i+2, changedRed, changedGreen, changedBlue);
      strip.setPixelColor(i+3, changedRed, changedGreen, changedBlue); 
    }
  }
  strip.show();
  delay(wait);
  }
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}


// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delC:\Users\Alex\Documents\GitHub\P2-Arduino-Codingay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void setColor(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
                               //  Pause for a moment
  }
  strip.show();                           //  Update strip to match
  delay(wait);
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

bool badTemp = false;

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue))); 
      /*Serial.println("Red, Green, Blue"); //If ya wanna see the colors of the Neo Pixels
      Serial.print(LEDr);
      Serial.print(", ");
      Serial.print(LEDg);
      Serial.print(", ");
      Serial.println(LEDb);
      delay(300);*/
    }

    CurrentTime = millis();
    if (CurrentTime >= (PreviousTime + 2000))
    {
      temperature = getTemp();
      Serial.print(temperature);
      Serial.println(" deg C");
      CurrentTime = millis();
      PreviousTime = CurrentTime;
    }
    
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
