#include "HUSKYLENS.h"


short yaw = 0, pitch = 0;
float pid[3] = { 1.5, 0.05, 1 };//PID Consts
unsigned long areaRange[2] = { 8100, 14400 };//Area Params
unsigned long int last_time = 0;
int pErr = 0;
unsigned short imgWdt = 320, imgHyt = 240;
int err, integral = 0, derivative, dt;
HUSKYLENS huskylens;

int controlSignal(unsigned int x, unsigned int y, unsigned int w, unsigned int h, float k_p, float k_i, float k_d, unsigned long dt, int pErr) {
  unsigned long a = w * h;
  if (a != 0) {
    if (a > areaRange[1]) pitch = -130;//backward speed
    else if (a < areaRange[0]) pitch = 150;//forward speed
    else if (a > areaRange[0] && a < areaRange[1]) pitch = 0;
    err = (x) - (int)imgWdt / 2;
    integral = integral + (err * dt);
    derivative = (err - pErr) / dt;
    yaw = (k_p * err) + (k_i * integral) + (k_d * derivative);
    yaw = yaw>180?180:yaw;
    yaw = yaw<-180?-180:yaw;
    pitch = pitch>180?180:pitch;
    pitch = pitch<-180?-180:pitch;
  }
  return err;
}

bool updateControls(HUSKYLENSResult result, unsigned int dt) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    unsigned int x = 0, y = 0, w = 0, h = 0, id = -1;
    x = result.xCenter;
    y = result.yCenter;
    w = result.width;
    h = result.height;
    id = result.ID;
    pErr = controlSignal(x, y, w, h, pid[0], pid[1], pid[2], dt, pErr);
  } else return false;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!huskylens.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
}

void loop() {
  unsigned long int now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      updateControls(result, dt);
      Serial.println(String() + F("pitch=") + pitch + F(",yaw=") + yaw);
    }
  }
}
















/*
void updateControls(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}
*/
