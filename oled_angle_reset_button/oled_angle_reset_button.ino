///////////////////////////  MPU6050 -- Angle Tilt  ///////////////////////////////
#include <Wire.h>              // Include Wire library for I2C communication
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

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);

#define xmax 128
#define ymax 64
#define xcenter xmax / 2
#define ycenter ymax / 2
#define arc_width xmax / 5  // ปรับค่าเพื่อเปลี่ยนความกว้างด้านข้างของครึ่งวงกลม
#define arc_height ymax / 5  // ปรับค่าเพื่อเปลี่ยนความสูงด้านข้างของครึ่งวงกลม

int p, w, m, a = 10;
int angle_value = 36;
int angle = 0;  // ตัวแปรมุม
char *label[] = {"LOAD", "COOLANT", "INTAKE", "VOLT"};
int labelXpos[] = {53, 45, 49, 53};
Adafruit_GFX_Button buttons[3];
uint8_t currButton = -1;

void gauge(uint8_t angle);

void setup() {
// Setup MPU6050
  Serial.begin(115200);
  Wire.begin();
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); 
///  End of setup MPU6050
  pinMode(8,INPUT);
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

int angle_define = 0,key = 0,check = 0;
void loop() {
//// First Impression MPU6050
//  if (check == 0){
//    setup_MPU6050();
//    check = 1;
//  }
////===========================  
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

  display.stopscroll();  // หยุดการกระพริบข้อความและเลข
  display.display(); // แสดงผลทั้งหมดครั้งเดียว
  read_circle_degree();  
  
//============  Button Reset Gauge ===================//
//  if (digitalRead(8) == 1 and key == 0){
//    display.clearDisplay();
//    display.display(); 
//    angle_define = 0;
//    key = 1;
//    delay(6000);
//  }
//  if (digitalRead(8) == 0 and key == 1){
//    key = 0;
//  }
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

//void setup_MPU6050(){
//  Wire.begin();
//  accelgyro.initialize();
//  Serial.println("Testing device connections...");
//  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");  
//}

void read_circle_degree(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//#ifdef OUTPUT_READABLE_ACCELGYRO
//  // display tab-separated accel/gyro x/y/z values
//  Serial.print("a/g:\t");
//  Serial.print(ax);
//  Serial.print("\t");
//  Serial.print(ay);
//  Serial.print("\t");
//  Serial.print(az);
//  Serial.print("\t");
//  Serial.print(gx);
//  Serial.print("\t");
//  Serial.print(gy);
//  Serial.print("\t");
//  Serial.println(gz);
//#endif

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
  if (x <= 90){ 
    degree_lockon = x ; 
  }
  else if (x > 90){
    degree_lockon = 90 ;     
  }
  delay(60);  
}
