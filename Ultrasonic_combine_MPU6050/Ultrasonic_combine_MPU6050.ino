//ขาที่ต่อ MPU6050  
//ขา SCL ของ MPU6050 Module ต่อเข้าขา SDA ของ Arduino 
//ขา SDA ของ MPU6050 Module ต่อเข้าขา SCL ของ Arduino 
//เเละก็มี VCC GND มีเเค่นี้เองครับการต่อ
///////////////////////////  GY-US42 -- Distance Sensor  ///////////////////////////////

#define SCL_PIN 5              // Default SDA is Pin 5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC
#define SDA_PIN 4              // Default SCL is Pin 4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC
#define I2C_TIMEOUT 100        // Define a timeout of 100 ms -- do not wait for clock stretching longer than this time

#include <SoftI2CMaster.h>

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

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");  
}

int start_distance=0 , start_angle=0;
void loop()
{
//  if (start_distance == 2){
//   Setup_GYUS42_distance();
//   start_distance = 1;
//  }
//  else if (start_angle == 0){
//   Setup_MPU6505_angle();
//   start_angle = 1;
//   
//  if(start_distance == 1){
//    read_the_sensor_example();
//  }
//  if(start_angle == 1){
    read_circle_degree();
//  }
//  delay(2000);
}

/////////////////////////// Setup MPU6050 (Angle Tilt)  ////////////////////////////


/////////////////////// Setup GY-US42 (Ultrasonic High Spec) ////////////////////
//void Setup_GYUS42_distance(){
//  i2c_init();
//  address_polling_example();
//  default_address_change_example();
//}
/////////////////////////////////////////////////////////////////////////////
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
//
//boolean start_sensor(byte bit8address)
//{
//  boolean errorlevel = 0;
//  bit8address = bit8address & B11111110;
//  errorlevel = !i2c_start(bit8address) || errorlevel;
//  errorlevel = !i2c_write(81) || errorlevel;
//  i2c_stop();
//  return errorlevel;
//}
//
//int read_sensor(byte bit8address)
//{
//  boolean errorlevel = 0;
//  int range = 0;
//  byte range_highbyte = 0;
//  byte range_lowbyte = 0;
//  bit8address = bit8address | B00000001;
//  errorlevel = !i2c_start(bit8address) || errorlevel;
//  range_highbyte = i2c_read(0);
//  range_lowbyte = i2c_read(1);
//  i2c_stop();
//  range = (range_highbyte * 256) + range_lowbyte;
//  if (errorlevel)
//  {
//    return 0;
//  }
//  else
//  {
//    return range;
//  }
//}
//
//boolean change_address(byte oldaddress, byte newaddress)
//{
//  boolean errorlevel = 0;
//  oldaddress = oldaddress & B11111110;
//  errorlevel = !i2c_start(oldaddress) || errorlevel;
//  errorlevel = !i2c_write(170) || errorlevel;
//  errorlevel = !i2c_write(165) || errorlevel;
//  errorlevel = !i2c_write(newaddress) || errorlevel;
//  i2c_stop();
//  return errorlevel;
//}
//
//void read_the_sensor_example()
//{
//  boolean error = 0;
//  int range;
//  error = start_sensor(224);
//  if (!error)
//  {
//    delay(100);
//    range = read_sensor(224);
//    Serial.print("R:");
//    Serial.println(range);
//  }
//}
//
//void address_polling_example()
//{
//  boolean error = 0;
//  int range = 0;
//  Serial.println("Polling addresses...");
//  for (byte i = 2; i != 0; i += 2)
//  {
//    error = 0;
//    error = start_sensor(i);
//    if (!error)
//    {
//      delay(100);
//      range = read_sensor(i);
//      Serial.println(i);
//      if (range != 0)
//      {
//        Serial.print("Device found at:");
//        Serial.print(i);
//        Serial.print(" Reported value of:");
//        Serial.println(range);
//      }
//    }
//    else
//    {
//      Serial.print("Couldn't start:");
//      Serial.println(i);
//    }
//  }
//  Serial.println("Address polling complete.");
//}
//
//void default_address_change_example()
//{
//  boolean error = 0;
//  int range;
//  Serial.println("Take a reading at the default address");
//  error = start_sensor(224);
//  if (!error)
//  {
//    delay(100);
//    range = read_sensor(224);
//    Serial.print("R:");
//    Serial.println(range);
//  }
//  Serial.println("Change the sensor at the default address to 222");
//  error = 0;
//  error = change_address(224, 222);
//  delay(200);
//  Serial.println("Take a reading at the new address");
//  error = 0;
//  error = start_sensor(222);
//  if (!error)
//  {
//    delay(100);
//    range = read_sensor(222);
//    Serial.print("N:");
//    Serial.println(range);
//  }
//  Serial.println("Change the sensor back to the default address");
//  error = 0;
//  error = change_address(222, 224);
//  delay(200);
//}
