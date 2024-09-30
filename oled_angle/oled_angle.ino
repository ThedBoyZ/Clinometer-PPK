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

int angle_define = 0,key = 0;;
void loop() {
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
  if (angle_define <= angle_value){
     angle_define += 1;
  }
  gauge(angle_define);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("angle :");

  display.setTextSize(2);
  display.setCursor(50, 2);
  display.print(angle_value);

  display.setTextSize(1);
  display.setCursor(75, 0);
  display.print("o");

  display.stopscroll();  // หยุดการกระพริบข้อความและเลข

  display.display(); // แสดงผลทั้งหมดครั้งเดียว
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
