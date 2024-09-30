/*                   Arduino I2C for a MaxSonar                         */
//////////////////////////////////////////////////////////////////////////
//  Arduino I2C for a MaxSonar by Carl Myhre is licensed under a        //
//  Creative Commons Attribution-ShareAlike 4.0 International License.  //
//  Original Author:  Carl Myhre, 10-02-2014, Revision: 1.0             //
//  Modifications by:                                                   //
//                                                                      //
//  Revision History: 1.0 -- 10-02-2014 -- Created initial code build   //
//                                                                      //
//  The original I2C libraries were created by Peter Fleury             //
//    http://homepage.hispeed.ch/peterfleury/avr-software.html          //
//                                                                      //
//  These libraries were adapted by Bernhard Nebel for use on Arduino   //
//    https://github.com/felias-fogg/SoftI2CMaster                      //
//                                                                      //
//  Special Thanks to MaxBotix Inc. for sponsoring this project!        //
//    http://www.maxbotix.com -- High Performance Ultrasonic Sensors    //
//                                                                      //
//  For more information on installing the I2C libraries for Arduino    //
//    visit http://playground.arduino.cc/Main/SoftwareI2CLibrary        //
//////////////////////////////////////////////////////////////////////////
//Hints on installing this code:
// 1. You will need to install the <SoftI2CMaster.h> library before using this code.
//      On Windows, the files are placed in C:\Program Files (x86)\Arduino\libraries\SoftI2CMaster\
// 2. As of 10-02-14 the Arduino library page (reference above) has the wrong name for the include file
//      it lists <SoftI2C.h> instead of <SoftI2CMaster.h> -- use the one that matches your installation.
// 3. Make sure to load the library into the Arduino compiler.
//      To do this go to: SKETCH >> IMPORT LIBRARY... >> ADD LIBRARY...
//      Then navigate to C:\Program Files (x86)\Arduino\libraries\SoftI2CMaster\SoftI2CMaster.h
// 4. Be sure to set the SCL and SDA pins so that they match the pins you are using.
// 5. I have included 3 working "code examples" which differ from the 3 "functions" I included.
//      The functions are all that should be required to quickly use the I2C library to talk to a MaxSonar.
//      The three code examples show how I would implement each of the common tasks you may wish to do.
// 6. The included functions are as follows:
//      A. start_sensor(addr)
//      B. read_sensor(addr)
//      C. change_address(oldaddr,newaddr)
// 7. The included code examples are as follows:
//      A. read_the_sensor_example()
//      B. address_polling_example()
//      C. default_address_change_example()
// 8. You do not have to keep track of the error codes passed out by the installed functions if you do not want to.
//      I inluded the error tracking so that it was easy for others to build a reliable system -- and to ease
//      troubleshooting. (not using it makes for cleaner code if you trust your interface)
/*
Below, I define the SCL and SDA pins by their ATMEGA pins I have included links to common mappings below.
    UNO:  http://arduino.cc/en/Hacking/PinMapping168
    NANO: (matches UNO but has fewer pins)
    MEGA 2560: http://arduino.cc/en/Hacking/PinMapping2560
The current data matches the setup for the Arduino Uno -- they may need to be changed if the hardware changes.
You can also switch the I2C interface
to any of the tristate pins that you want (not just the SDA or SCL pins).
*/
#define SCL_PIN 3              //Default SDA is Pin5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC
#define SDA_PIN 2              //Default SCL is Pin4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC
#define I2C_TIMEOUT 100        //Define a timeout of 100 ms -- do not wait for clock stretching longer than this time
/*
I have included a couple of extra useful settings for easy reference.
//#define I2C_CPUFREQ (F_CPU/8)//Useful if you plan on doing any clock switching
#define I2C_FASTMODE 1         //Run in fast mode (400 kHz)
#define I2C_SLOWMODE 1         //If you do not define the mode it will run at 100kHz with this define set to 1 it will run at 25kHz
*/
#include <SoftI2CMaster.h>     //You will need to install this library
#include <SPI.h>
// ======================================== end of Distance sensor GY-US42 ===============================================

#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Constants for OLED display
#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(OLED_RESET);

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ ------ Calcute Tree height ------- ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]//
#include <Arduino.h>
#include <math.h>

const float MY_PI = 3.14159265;
float Tree_Height = 0;
float Lockon_Tree_Distance_To_Me = 0;
int Lockon_Angle_Degree = 0;

//========================================== Button state Mode ==========================================//
int button_mode = 1;
int key = 0;

//========================================== end of  Button =============================================// 

//================================= OLED screen 1: Time when home page =========================================//
static int counter = 0; 


// OLED screen 2: Distance input page
int check = 0;
int range = 0;
float distance_r = 0;
float c = 0;
int i = 0;
int l = 1;
int mem_length_l = 1;
const int MAX_WIDTH = 120;  // Maximum width for the rectangles

//================================= OLED screen 3: Angle tilt input page =======================================//
///////////////////////////  MPU6050 -- Angle Tilt  ///////////////////////////////
#include <Wire.h>
#include <MPU6050.h>           // Include MPU6050 library for interacting with the sensor

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
bool blinkState = false;
const int MPU_addr = 0x68;
int16_t axis_X, axis_Y, axis_Z;

int minVal = 265;
int maxVal = 402;
int degree_lockon = 0; // save variable Real-time for MPU6050 / Gyroscope senser

double x;
double y;
double z;
/////////////////////////// - end Of Angle Sensor - /////////////////////////////

#define xmax 128
#define ymax 64
#define xcenter xmax / 2
#define ycenter ymax / 2
#define arc_width xmax / 5  // ปรับค่าเพื่อเปลี่ยนความกว้างด้านข้างของครึ่งวงกลม
#define arc_height ymax / 5  // ปรับค่าเพื่อเปลี่ยนความสูงด้านข้างของครึ่งวงกลม

int p, w, m, a = 10;
//int angle_value = 36;
int angle_define = 0;
int angle = 0;  // ตัวแปรมุม
char *label[] = {"LOAD", "COOLANT", "INTAKE", "VOLT"};
int labelXpos[] = {53, 45, 49, 53};
Adafruit_GFX_Button buttons[3];
uint8_t currButton = -1;
//=================================== Add for MPU6050 ========================================//
void read_circle_degree(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write((uint8_t)(ax >> 8));
  Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8));
  Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8));
  Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8));
  Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8));
  Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8));
  Serial.write((uint8_t)(gz & 0xFF));
#endif

  axis_X = Wire.read() << 8 | Wire.read();
  axis_Y = Wire.read() << 8 | Wire.read();
  axis_Z = Wire.read() << 8 | Wire.read();

  int xAng = map(ax, minVal, maxVal, -90, 90);
  int yAng = map(ay, minVal, maxVal, -90, 90);
  int zAng = map(az, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  Serial.print("Angle : ");
  Serial.println(x);
  if (90-x <= 90 && 90-x >= 0){ 
    degree_lockon = 90-x ; 
  }
  else{
    degree_lockon = 90 ;     
  }
  delay(60);  
}

void gauge(uint8_t angle);

//========================================== end of OLED screen 3 ==============================================// 

// 'tree_1', 128x64px
const unsigned char myBitmap [] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb3, 0x7f, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xc9, 0x6f, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xa4, 0xd5, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0x0d, 0x77, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf5, 0xf7, 0xd0, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9d, 0x79, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe9, 0x46, 0x2d, 0xbf, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc6, 0x33, 0x04, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd6, 0x9b, 0x41, 0xbf, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x78, 0xb2, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfa, 0x61, 0x09, 0x3f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0x00, 0x0c, 0xbf, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcb, 0x20, 0x5b, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0x41, 0xfd, 0x7f, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xf6, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x7f, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// Helper functions for OLED screen 1

void setup_screen1() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

// ======================================= OLED functions for screen 1 --Homepage =======================================//
void loop_screen1() {
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
  display.setCursor(xPos, 8); 
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
  display.setCursor(10, 25); // Custom position for the logo
  display.print("Clinometer");
  
  display.display(); 
}

// ======================================= OLED functions for screen 2 --Distance Progress =======================================//
void setup_screen2() {
  i2c_init();
  // (OPTIONAL) Check each address for a sensor
  address_polling_example();
  /*
    Note that I placed the address change example in setup() for a good reason.
    Changing the sensor address causes an EEPROM write, there should only be ~1,000,000+
      of these writes to the sensor microcontroller over its product lifetime.
    Changing the address is fine, but doing it every second for the next 4 years may
      cause reliability issues.
  */
  // (OPTIONAL) Run an address change example
  default_address_change_example();  
  delay(200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  // Initialize both the serial and I2C bus
  DrawTitles();
}

void loop_screen2() {
  //read_the_sensor_example();
  c = distance_r * 17;
  l = distance_r * 13/7.2;
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(35, 10);  // Adjusted y-coordinate to move the random number line up
  display.println(distance_r);
  display.setCursor(100, 10);
  display.println("m");
  display.display();
  int rectWidth = constrain(128 - c, 0, MAX_WIDTH);  // Limit the width of the rectangles
  display.fillRect(c, 50, rectWidth, 8, BLACK);
  display.fillRect(3, 50, c, 8, WHITE);
  display.display();
//    display.display();
//
//  Serial.print("Mem = ");
//  Serial.println(mem_length_l);
//  Serial.print("===================================");
  i = mem_length_l;
  while(i!=l){
      if (i <= l){
        display.fillRect(i * 10, 28, 1, 8, WHITE);
        i++;
      }
      if (i > l){        
        display.fillRect(i * 10, 28, 1, 8, BLACK);
        i--;
      }
      delay(40);
      display.display();
      }
      
  mem_length_l = l;
  delay(200);  // Delay between updates
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

///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//  had an error, 0 = the function was successful
boolean start_sensor(byte bit8address){
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}
///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error

/////////////////////////////////////////
// Function: Change the sensor address //
/////////////////////////////////////////
//Uses the I2C library to change the address of a sensor at a given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte oldaddress = the current address of the sensor that we want to change
//INPUTS: byte newddress  = the address that we want to change the sensor to
//OUPUTS: bit  errorlevel = reports if the function was successful in changing the address: 1 = the function had an
//      error, 0 = the function was successful
boolean change_address(byte oldaddress,byte newaddress){
  //note that the new address will only work as an even number (odd numbers will round down)
  boolean errorlevel = 0;
  oldaddress = oldaddress & B11111110;  //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(oldaddress) | errorlevel; //Start communication at the new address and track error codes
  errorlevel = !i2c_write(170) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(165) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(newaddress) | errorlevel; //Send the new address
  i2c_stop();
  return errorlevel;
}
//////////////////////////////////////////////////////////
// Code Example: Read the sensor at the default address //
//////////////////////////////////////////////////////////
void read_the_sensor_example(){
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  float range;
  //Take a range reading at the default address of 224
  error = start_sensor(224);    //Start the sensor and collect any error codes.
  if (!error){                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    range = read_sensor(224);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("R:");Serial.println(range);
    distance_r = range/100;
  }
}

////////////////////////////////////////////////////////////////
// Code Example: Poll all possible addresses to find a sensor //
////////////////////////////////////////////////////////////////
void address_polling_example(){
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  int range = 0;
  Serial.println("Polling addresses...");
  //Walk through all possible addresses and check for a device that can receive the range command and will
  //    return two bytes.
  for (byte i=2; i!=0; i+=2){   //start at 2 and count up by 2 until wrapping to 0. Checks all addresses (2-254) except 0 (which cannot be used by a device)
    error = 0;
    error = start_sensor(i);    //Start the sensor and collect any error codes.
    if (!error){                //If you had an error starting the sensor there is little point in reading it.
      delay(100);
      range = read_sensor(i);   //reading the sensor will return an integer value -- if this value is 0 there was an error
      Serial.println(i);
      if (range != 0){
        Serial.print("Device found at:");Serial.print(i);Serial.print(" Reported value of:");Serial.println(range);
      }  
    }
    else{
      Serial.print("Couldn't start:");Serial.println(i);
    }
  }
  Serial.println("Address polling complete.");
}
//////////////////////////////////////////////
// Code Example: Change the default address //
//////////////////////////////////////////////
void default_address_change_example(){
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  int range;
  Serial.println("Take a reading at the default address");
  //Take a range reading at the default address of 224
  error = start_sensor(224);    //Start the sensor and collect any error codes.
  if (!error){                  //If you had an error starting the sensor there is little point in reading it.
    delay(100);
    range = read_sensor(224);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("R:");Serial.println(range);
    distance_r = range;
  }
   Serial.println("Change the sensor at the default address to 222");
  //Change the address from 224 to 222
  error = 0;
  error = change_address(224,222);  //Change the address -- I don't do anything with the error handler at this point but you can if you want.
  delay(200);    //Wait 125ms for the sensor to save the new address and reset
   Serial.println("Take a reading at the new address");
  //Take a range reading at the new address of 222
  error = 0;
  error = start_sensor(222);     //Same as above but at the new address
  if (!error){
    delay(100);
    range = read_sensor(222);
    Serial.print("N:");Serial.println(range);
  }  
   Serial.println("Change the sensor back to the default address");  
  //Change the address from 222 to 224
  error = 0;
  error = change_address(222,224);
  delay(200);    //Wait 125ms for the sensor to save the new address and reset
}

// ======================================= OLED functions for screen3 --Angle Tilt Progress =======================================//
void setup_screen3() {
// Setup MPU6050
  Wire.begin();
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); 
///  End of setup MPU6050
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  display.clearDisplay();

  display.drawCircle(xcenter, ycenter, arc_width, WHITE);
  display.drawCircle(xcenter, ycenter, arc_width, WHITE);

  display.setCursor(labelXpos[1], 30);
  display.setTextSize(2);
  display.print(label[1]);

  display.display();
}

void loop_screen3() {
// First Impression MPU6050
  if (check == 0){
    setup_MPU6050();
    check = 1;
  }
//===========================  
  p += a;

  if (p > 1023)
    a = -10;
  else if (p < 0)
    a = 10;

  w = map(p, 0, 1023, 0, 100);
  m = map(p, 0, 1023, 0, 90);

  uint8_t xx = m;
  if (xx < 45) {
    xx += 135;
  } else {
    xx -= 45;
  }

  display.clearDisplay();
  if (angle_define < degree_lockon){
     angle_define += 1;
  }
  else if (angle_define > degree_lockon){
     angle_define -=1;
  }
  gauge(angle_define);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("angle :");
  
  display.setTextSize(2);
  display.setCursor(50, 2);
  display.print(degree_lockon);

  display.setTextSize(1);
  display.setCursor(75, 0);
  display.print("o");
  display.setCursor(100, 0);
  float angleDegrees = degree_lockon; // Angle in degrees
  float angleRadians = angleDegrees * (MY_PI / 180); // Convert degrees to radians
  if (degree_lockon < 90){
  display.print(tan(angleRadians));
  }
  else{
  display.setCursor(90, 0);
  display.print("1.63e+16");    
  }
  display.stopscroll();  // หยุดการกระพริบข้อความและเลข
  display.display(); // แสดงผลทั้งหมดครั้งเดียว
  read_circle_degree();  
  
//============  Button Reset Gauge ===================//
  if (digitalRead(8) == 1 and key == 0){
    display.clearDisplay();
    display.display(); 
    angle_define = 0;
    key = 1;
    delay(6000);
  }
  if (digitalRead(8) == 0 and key == 1){
    key = 0;
  }
//====================================================//
}

void gauge(uint8_t angle) {
  display.drawCircle(xcenter, ycenter - 8 + arc_height + 45, arc_width + 35, WHITE);

  // ตำแหน่งฐานเข็มปัด
  int baseX = xcenter;
  int baseY = ycenter;

  // วงกลมฐานเข็มปัด
  display.fillCircle(baseX, baseY, 3, WHITE);

  // คำนวณตำแหน่งปลายเข็มปัด
  float x1 = sin(((angle+10)-90) * 2 * PI / 360);
  float y1 = cos(((angle+10)-90) * 2 * PI / 360);
  int x2 = baseX + (arc_width * x1);
  int y2 = baseY - (arc_width * y1);

  // เส้นเชื่อมต่อฐานและปลายเข็มปัด
  display.drawLine(baseX, baseY, x2, y2 + 10, WHITE);
}
// ======================================= OLED functions for screen4 --Success Length Tree Progress =======================================//
void setup_screen4() {
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay(); //for Clearing the display
  display.drawBitmap(0, 0, myBitmap, 128, 64, WHITE); // display.drawBitmap(x position, y position, bitmap data, bitmap width, bitmap height, color)
  display.fillRect(0, 0, 74, 22, SSD1306_BLACK); // Fill a rectangle at position (10, 30) with width 30 and height 20 in black color
  display.fillTriangle(74, 16, 74, 22, 60, 22, SSD1306_WHITE);
//  display.fillRect(4, 4, 64, 19, SSD1306_WHITE); // Fill a rectangle at position (10, 30) with width 30 and height 20 in black color
  display.setTextColor(SSD1306_WHITE); // Set text color to black
  display.setTextSize(2);
  display.setCursor(6, 3);

  //====================================  tan theta Concept ====================================//
  float angleDegrees = Lockon_Angle_Degree; // Angle in degrees
  float angleRadians = angleDegrees * (MY_PI / 180); // Convert degrees to radians
  if (Lockon_Angle_Degree < 90){
    Tree_Height = Lockon_Tree_Distance_To_Me * tan(angleRadians);
    display.print(Tree_Height);
  }
  else{
    display.print("None");    
  }

  //=============================================================================================
  display.setTextColor(SSD1306_BLACK); // Set text color to black
  display.setTextSize(2);
  display.setCursor(79, 3);
  display.print("m");
  display.fillRect(10, 23, 8, 7, SSD1306_BLACK); // Fill a rectangle at position (10, 30) with width 30 and height 20 in black color
  display.display();
  delay(700);
  display.fillRect(25, 23, 8, 7, SSD1306_BLACK); // Fill a rectangle at position (25, 30, Weight, Height)
  display.display();
  delay(700);
  display.fillRect(40, 23, 8, 7, SSD1306_BLACK); // Fill a rectangle at position (40, 30) 
  display.display();
  delay(700);
  display.fillRect(55, 23, 8, 7, SSD1306_BLACK); // Fill a rectangle at position (55, 30)
  display.display();
  delay(700);
  display.display();
}

void loop_screen4() {
}

// ======================================= Button Repeat Can Modify Mode Function ====================================//
void buttton_state_mode() {
  if (digitalRead(8) == 1 && key == 0) {
    if (button_mode < 5) {
      button_mode += 1;
    }
    if (button_mode >= 5) {
      button_mode = 1;
    }
    Serial.print("Button = ");
    Serial.println(button_mode);
    Serial.println("-----------------------------------");
    // Setup new Screen
    if (button_mode == 1) {
      Adafruit_SSD1306 display(4);
      setup_screen1();
    } else if (button_mode == 2) {
      check = 0;
      Adafruit_SSD1306 display(4);
      setup_screen2();
    } else if (button_mode == 3) {
      Lockon_Tree_Distance_To_Me = distance_r;
      angle_define = 0;
      Adafruit_SSD1306 display(0);
      setup_screen3();
    } else{
      Lockon_Angle_Degree = degree_lockon;
      Adafruit_SSD1306 display(-1);
      setup_screen4();
    }
    key = 1;
  }
  if (digitalRead(8) == 0 && key == 1) {
    key = 0;
  }
  delay(200);
}

// ======================================= Setup Device ====================================//
void setup() {
  // Button Mode
  Serial.begin(115200);
  pinMode(9,INPUT);
  // Call setup functions for each screen
  setup_screen1();
  setup_screen2();
  setup_screen3();
  setup_screen4();
}

void loop() {
  if (Serial.available() > 0){
    button_mode = Serial.parseInt();
    if (button_mode == 1) {
      loop_screen1();
    } else if (button_mode == 2) {
      loop_screen2();
    } else if (button_mode == 3) {
      loop_screen3();
    } else if (button_mode == 4) {
      loop_screen4();
    }
  }

  if(digitalRead(8) == 1 && key == 0){
    buttton_state_mode(); // Check button state for mode change
    key = 1;
  }
  else if (digitalRead(8) == 0 and key == 1){
    key = 0;
 }
}
