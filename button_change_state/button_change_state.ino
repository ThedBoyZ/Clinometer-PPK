int button_mode = 1;

void setup() {
 Serial.begin(115200);
 pinMode(8,INPUT);
}

int key = 0;

void loop() {
  if (digitalRead(8) == 1 and key == 0)
  {
    if (button_mode < 4){
      button_mode += 1;
    }
    else{
      button_mode = 1;
    }
    Serial.print("Button = ");
    Serial.println(button_mode);
    Serial.println("-----------------------------------");
    key = 1;
  }
  if (digitalRead(8) == 0 and key == 1)
  {
    key = 0;
  }
  delay(200);
}
