#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// OLED screen 2: Distance input page
int r = 0;
float c = 0;
int i = 0;
int l = 0;
const int MAX_WIDTH = 120;  // Maximum width for the rectangles

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  display.clearDisplay();
  display.display();
  delay(1000);

  DrawTitles();
}

void loop() {
  r = 4.5;
  c = r * 17;
  l = r*13/7.2;
  display.setTextSize(3);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(20, 20);  // Adjusted y-coordinate to move the random number line up
  display.println(r);
  display.setCursor(100, 20);
  display.println("m");
//  display.display();
//  int rectWidth = constrain(128 - c, 0, MAX_WIDTH);  // Limit the width of the rectangles
//  display.fillRect(c, 50, rectWidth, 8, BLACK);
//  display.fillRect(3, 50, c, 8, WHITE);
//  display.display();

    for (i = 1; i < l; i++) {
      display.fillRect(i * 10, 50, 1, 8, WHITE);
      delay(700);
      display.display();
  }
   
}

void DrawTitles(void) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Distance Tree :");
  display.display();
}

String Format(double val, int dec, int dig ) {
  int addpad = 0;
  char sbuf[20];
  String fdata = (dtostrf(val, dec, dig, sbuf));
  int slen = fdata.length();
  for ( addpad = 1; addpad <= dec + dig - slen; addpad++) {
    fdata = " " + fdata;
  }
  return fdata;
}
