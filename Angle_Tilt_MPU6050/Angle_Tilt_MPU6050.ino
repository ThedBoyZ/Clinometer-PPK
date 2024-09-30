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

double x;
double y;
double z;
/////////////////////////// - end Of Angle Sensor - /////////////////////////////
void setup() {
  Serial.begin(115200);
  Wire.begin();
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");  
}

void loop() {
    read_circle_degree();
}

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
  delay(100);  
}
