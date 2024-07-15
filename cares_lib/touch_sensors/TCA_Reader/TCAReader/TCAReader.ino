#include <Arduino_BuiltIn.h>

/*
 * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

 /*
 Always goes 5v -> Ground -> SCL -> SDA
 */

#include "Wire.h"
#include "SPL06-007.h"
#include <Adafruit_NeoPixel.h>

#define TCAADDR 0x70

#define PIN 6
#define NUMPIXELS 24
#define DELAYVAL 100
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int Ports[2][8] = {{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1}};
float Pressure_Baseline[1][8] = {0,0,0,0,0,0,0,0};

#define BAUDRATE 921600

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


// standard Arduino setup()
void setup()
{
    int add_count = 0;
    int temp;

    //LED Pin Setup
    pinMode(LED_BUILTIN, OUTPUT);
    //Neopixel initialisation
    pixels.begin();
    for(int i=0; i<NUMPIXELS; i++) { 
      pixels.setPixelColor(i, pixels.Color(255, 255, 255));
      pixels.setBrightness(100);
      pixels.show();  
      delay(DELAYVAL); 
    }
    // pixels.clear();
    pixels.show();

    while (!Serial);
    delay(1000);

    Wire.begin();
    
    Serial.begin(BAUDRATE);
    Serial.println("\nTCAScanner ready!");
    Serial.setTimeout(30); //50ms as pressure printing takes ~24ms
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
          Ports[0][t] = t;        //Ports[0][] stores the multiplexer port number
          Ports[1][t] = addr,HEX; //Ports[1][] stores the I2C address of the barometic sensors 
        }
      }
      delay(10);
    }
    Serial.println("Search Complete\n");

    delay(1000);
    for(int i=0;i<8;i++){
      if(Ports[0][i] != -1){
        tcaselect(Ports[0][i]);
        Wire.begin();    // begin Wire(I2C)
        Serial.begin(BAUDRATE); // begin Serial
        char buffer[30];
        Serial.print("Goertek-SPL06-007: Setting up Sensor @ Port ");
        Serial.print(Ports[0][i]);
        Serial.print(" with address 0x");
        Serial.println(Ports[1][i],HEX);
        SPL_init(); // Setup initial SPL chip registers
      }
    }
    generatePressureBaseline();
    Serial.print("Pressure Baselines: ");
    for(int i=0; i<8;i++){
        Serial.print(Pressure_Baseline[0][i]);
        Serial.print("  ");
    }
    Serial.print("\n");
    Serial.print("Begin:");
    Serial.print("\n");
}

uint8_t generatePressureBaseline(){
  float temp;
  for(int i=0; i<8; i++){
      if(Ports[0][i] != -1){
          tcaselect(Ports[0][i]);
          temp = get_pressure();
          delay(10);
          temp = get_pressure();
          Pressure_Baseline[0][i] = temp;
      }
  }
  return Pressure_Baseline;
}

void printpressures(){
  for(int i=0;i<8;i++){
    if(Ports[0][i] != -1){
      tcaselect(Ports[0][i]);
      Serial.print(get_pressure(),2);
      Serial.print(" ");
    } else {
      continue;
    }
  }
  Serial.println(" ");
  // pixels.clear();
  pixels.show();
}

void (* resetFunc) (void) = 0;

void quit(){
  Serial.println("quit");
}

void flashLED(){
  for(int i=0; i<NUMPIXELS-7; i++) { 
      pixels.setPixelColor(i, pixels.Color(255, 255, 255));
      pixels.setBrightness(10);
      pixels.show();  
    }
}

void loop() 
{
  if (Serial.available() > 0){
    String msg = Serial.readString();
    if (msg == "exit"){
      quit();
    }
  }
  printpressures();
}



