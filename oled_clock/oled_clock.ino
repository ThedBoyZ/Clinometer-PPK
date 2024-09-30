#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(OLED_RESET);

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  static int counter = 0; 
  int seconds = counter % 60; // Calculate seconds 
  int minutes = (counter / 60) % 60; // Calculate minutes
  int hours = counter / 3600; // Calculate hours 
  
  displayTime(hours, minutes, seconds);
  delay(1000);
  counter++; // Increment the counter to keep track of time
}

void displayTime(int hours, int minutes, int seconds) {
  display.clearDisplay(); // Clear the display buffer

  // Calculate the horizontal position to center the time string
  int xPos = (SCREEN_WIDTH - 84) / 2; // Center the time display (84 is the total width of "00:00:00")
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(xPos, 9); 
  display.print(hours);
  display.print(":");
  if (minutes < 10) {
    display.print("0"); // Add leading zero for single-digit minutes
  }
  display.print(minutes);
  display.print(":");
  if (seconds < 10) {
    display.print("0"); // Add leading zero for single-digit seconds
  }
  display.print(seconds);

  // Set the position for the "Clinometer" logo
  display.setTextSize(1);
  display.setCursor(10, 27); // Custom position for the logo
  display.print("Clinometer");
  
  display.display(); 
}
