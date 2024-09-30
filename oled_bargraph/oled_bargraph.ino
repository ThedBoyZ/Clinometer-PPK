#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 Display(OLED_RESET);

float r = 0;
float c = 0;
int i = 0;

const int MAX_WIDTH = 120;  // Maximum width for the rectangles

void setup() {
  Display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  Display.clearDisplay();
  Display.display();
  delay(1000);

  DrawTitles();
}

void loop() {
  r = 4.2;
  c = r * 17;
  Display.setTextSize(2);
  Display.setTextColor(WHITE, BLACK);
  Display.setCursor(35, 12);  // Adjusted y-coordinate to move the random number line up
  Display.println(r);
  Display.setCursor(90, 12);
  Display.println("m");
  int rectWidth = constrain(128 - c, 0, MAX_WIDTH);  // Limit the width of the rectangles
  Display.fillRect(c, 30, rectWidth, 8, BLACK);
  Display.fillRect(3, 30, c, 8, WHITE);

  for (i = 1; i < 13; i++) {
    Display.fillRect(i * 10, 30, 1, 8, BLACK);
  }

  Display.display();
  delay(1000);  // Delay between updates
}

void DrawTitles(void) {
  Display.setTextSize(1);
  Display.setTextColor(WHITE);
  Display.setCursor(0, 0);
  Display.println("Distance Tree :");
  Display.display();
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
