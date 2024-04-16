#include "HUSKYLENS.h"
#include <WiFi.h>
#include <Servo.h>
#include "Button2.h"
#include "Adafruit_VL53L0X.h"

// Right Front Motor
#define MF_AI1 6
#define MF_AI2 7

// Left Front Motor
#define MF_BI1 8
#define MF_BI2 9

// Right Rear Motor
#define MR_AI1 10
#define MR_AI2 11

// Left Rear Motor
#define MR_BI1 12
#define MR_BI2 13

#define BUTTON_PIN 14
#define TOTAL_MODE 4
#define MIN_IN_MM 250

// Test speed for all motors (change as desired)
int rf_PWM = 175;  // Right Front Motor
int lf_PWM = 180;  // Left Front Motor
int rr_PWM = 220;  // Right Rear Motor
int lr_PWM = 220;  // Left Front Motor

const byte MEC_STOP = 0b00000000;
const byte MEC_STRAIGHT_FORWARD = 0b10101010;
const byte MEC_STRAIGHT_BACKWARD = 0b01010101;
const byte MEC_SIDEWAYS_RIGHT = 0b01101001;
const byte MEC_SIDEWAYS_LEFT = 0b10010110;
const byte MEC_DIAGONAL_45 = 0b00101000;
const byte MEC_DIAGONAL_135 = 0b10000010;
const byte MEC_DIAGONAL_225 = 0b00010100;
const byte MEC_DIAGONAL_315 = 0b01000001;
const byte MEC_PIVOT_RIGHT_FORWARD = 0b00100010;
const byte MEC_PIVOT_RIGHT_BACKWARD = 0b00010001;
const byte MEC_PIVOT_LEFT_FORWARD = 0b10001000;
const byte MEC_PIVOT_LEFT_BACKWARD = 0b01000100;
const byte MEC_ROTATE_CLOCKWISE = 0b01100110;
const byte MEC_ROTATE_COUNTERCLOCKWISE = 0b10011001;
const byte MEC_PIVOT_SIDEWAYS_FRONT_RIGHT = 0b01100000;
const byte MEC_PIVOT_SIDEWAYS_FRONT_LEFT = 0b10010000;
const byte MEC_PIVOT_SIDEWAYS_REAR_RIGHT = 0b00001001;
const byte MEC_PIVOT_SIDEWAYS_REAR_LEFT = 0b00000110;

short yaw = 0, pitch = 0, throttle = 0;
float pid[3] = { 0.8, 0.05, 0.4 };             //PID Consts{0.8,0.05,0.4}
unsigned long areaRange[2] = { 8100, 14400 };  //Area Params
unsigned long int last_time = 0;
int pErr_xy = 0, pErr_z = 0;
unsigned short imgWdt = 320, imgHyt = 240;
int err, integral = 0, derivative, integral_z = 0, derivative_z, dt;
short mode = 1;

const char* ssid = "Phoenix";
const char* passssword = "dobbyisfree";

struct ERROR {
  int pErr_xy;
  int pErr_z;
};

HUSKYLENS huskylens;
WiFiServer server(80);
Button2 button;
Adafruit_VL53L0X lox;
Servo gimble;

void moveMotors(unsigned short speedRF, unsigned short speedLF, unsigned short speedRR, unsigned short speedLR, byte dircontrol) {
  // Right Front Motor
  analogWrite(MF_AI1, bitRead(dircontrol, 7) * speedRF);
  analogWrite(MF_AI2, bitRead(dircontrol, 6) * speedRF);

  // Left Front Motor
  analogWrite(MF_BI1, bitRead(dircontrol, 5) * speedLF);
  analogWrite(MF_BI2, bitRead(dircontrol, 4) * speedLF);

  // Right Rear Motor
  analogWrite(MR_AI1, bitRead(dircontrol, 3) * speedRR);
  analogWrite(MR_AI2, bitRead(dircontrol, 2) * speedRR);

  // Left Rear Motor
  analogWrite(MR_BI1, bitRead(dircontrol, 1) * speedLR);
  analogWrite(MR_BI2, bitRead(dircontrol, 0) * speedLR);
}


ERROR controlSignal(unsigned int x, unsigned int y, unsigned int w, unsigned int h, float k_p, float k_i, float k_d, unsigned long dt, int pErr_xy, int pErr_z) {
  unsigned long a = w * h;
  struct ERROR error;
  if (a != 0) {
    if (a > areaRange[1]) pitch = -180;      //backward speed
    else if (a < areaRange[0]) pitch = 180;  //forward speed
    else if (a > areaRange[0] && a < areaRange[1]) pitch = 0;
    error.pErr_xy = (x) - (int)imgWdt / 2;
    integral = integral + (error.pErr_xy * dt);
    derivative = (error.pErr_xy - pErr_xy) / dt;
    yaw = (k_p * error.pErr_xy) + (k_i * integral) + (k_d * derivative);
    error.pErr_z = (y) - (int)imgHyt / 2;
    integral_z = integral_z + (error.pErr_z * dt);
    derivative_z = (error.pErr_z - pErr_z) / dt;
    throttle = (k_p * error.pErr_z) + (k_i * integral_z) + (k_d * derivative_z);
    throttle = -throttle;
    yaw = yaw > 180 ? 180 : yaw;
    yaw = yaw < -180 ? -180 : yaw;
    pitch = pitch > 180 ? 180 : pitch;
    pitch = pitch < -180 ? -180 : pitch;
    throttle = throttle > 120 ? 120 : throttle;
    throttle = throttle < -120 ? -120 : throttle;
  }
  return error;
}

bool updateControls(HUSKYLENSResult result, unsigned int dt) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    unsigned int x = 0, y = 0, w = 0, h = 0, id = -1;
    struct ERROR getErr;
    x = result.xCenter;
    y = result.yCenter;
    w = result.width;
    h = result.height;
    id = result.ID;
    getErr = controlSignal(x, y, w, h, pid[0], pid[1], pid[2], dt, pErr_xy, pErr_z);
    pErr_xy = getErr.pErr_xy;
    pErr_z = getErr.pErr_z;
  } else return false;
  return true;
}

void stop() {
  digitalWrite(MF_AI1, LOW);
  digitalWrite(MF_AI2, LOW);
  digitalWrite(MF_BI1, LOW);
  digitalWrite(MF_BI2, LOW);
  digitalWrite(MR_AI1, LOW);
  digitalWrite(MR_AI2, LOW);
  digitalWrite(MR_BI1, LOW);
  digitalWrite(MR_BI2, LOW);
}

void motorsPinSet() {
  pinMode(MF_AI1, OUTPUT);
  digitalWrite(MF_AI1, LOW);
  pinMode(MF_AI2, OUTPUT);
  digitalWrite(MF_AI2, LOW);
  pinMode(MF_BI1, OUTPUT);
  digitalWrite(MF_BI1, LOW);
  pinMode(MF_BI2, OUTPUT);
  digitalWrite(MF_BI2, LOW);
  pinMode(MR_AI1, OUTPUT);
  digitalWrite(MR_AI1, LOW);
  pinMode(MR_AI2, OUTPUT);
  digitalWrite(MR_AI2, LOW);
  pinMode(MR_BI1, OUTPUT);
  digitalWrite(MR_BI1, LOW);
  pinMode(MR_BI2, OUTPUT);
  digitalWrite(MR_BI2, LOW);
}

void autoMove(int pitch, int yaw, int offsetRF = 0, int offsetLF = 0, int offsetRR = 0, int offsetLR = 0, int max8bitSpeed = 150, int pitchRange = 100, int yawRange = 100) {
  yaw = yaw > yawRange ? yawRange : yaw;
  yaw = yaw < -yawRange ? -yawRange : yaw;
  pitch = pitch > pitchRange ? pitchRange : pitch;
  pitch = pitch < -pitchRange ? -pitchRange : pitch;
  pitch = map(pitch, -pitchRange, pitchRange, -max8bitSpeed + abs(yaw), max8bitSpeed - abs(yaw));
  //Serial.println(String() + F("pitch=") + pitch + F(",yaw=") + yaw);
  if (pitch > 0) {
    analogWrite(MF_AI1, abs((pitch - yaw) + offsetRF));
    analogWrite(MF_AI2, 0);
    analogWrite(MF_BI1, abs((pitch + yaw) + offsetLF));
    analogWrite(MF_BI2, 0);
    analogWrite(MR_AI1, abs((pitch - yaw) + offsetRR));
    analogWrite(MR_AI2, 0);
    analogWrite(MR_BI1, abs((pitch + yaw) + offsetLR));
    analogWrite(MR_BI2, 0);
  } else if (pitch < 0) {
    analogWrite(MF_AI1, 0);
    analogWrite(MF_AI2, abs(pitch - yaw) + offsetRF);
    analogWrite(MF_BI1, 0);
    analogWrite(MF_BI2, abs(pitch + yaw) + offsetLF);
    analogWrite(MR_AI1, 0);
    analogWrite(MR_AI2, abs(pitch - yaw) + offsetRR);
    analogWrite(MR_BI1, 0);
    analogWrite(MR_BI2, abs(pitch + yaw) + offsetLR);
  } else {
    stop();
  }
}

void wifiMode() {
  WiFiClient client = server.accept();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no\" /><style>.skin1 button{border: 3px solid #fe53bb;color: #fe53bb;background-color: #840151;}.skin1 button:hover {border-color: #fe86cf;color: #fe86cf;background-color: #b70170;}.skin1 button:focus {border-color: #fe86cf;color: #fe86cf;background-color: #b70170;padding: 16px 16px;outline: 0;border-radius: 12px;}.skin1 button:focus:first-of-type {border-radius: 30px 12px 12px 30px;}.skin1 button:focus:last-of-type{border-radius: 12px 30px 30px 12px;}.skin2 button{border: 3px solid #f5d300;color: #f5d300;background-color: #292300;}.skin2 button:hover{border-color: #ffe129;color: #ffe129;background-color: #5c4f00;}skin2 button:focus {border-color: #ffe129;color: #ffe129;background-color: #5c4f00;padding: 16px 16px;outline: 0;border-radius: 12px;}.skin2 button:focus:first-of-type {border-radius: 30px 12px 12px 30px;}.skin2 button:focus:last-of-type{border-radius: 12px 30px 30px 12px;}.skin3 button {border: 3px solid #00d4ff;color: #00d4ff;background-color: #002a33;}.skin3 button:hover{border-color: #33ddff;color: #33ddff;background-color: #005566;}.skin3 button:focus {border-color: #33ddff;color: #33ddff;background-color: #005566;padding: 16px 16px;outline: 0;border-radius: 12px;}.skin3 button:focus:first-of-type{border-radius: 30px 12px 12px 30px;}.skin3 button:focus:last-of-type{border-radius: 12px 30px 30px 12px;}.skin4 button {border: 3px solid #72ff00;color: #72ff00;background-color: #173300;}.skin4 button:hover {border-color: #8eff33;color: #8eff33;background-color: #2e6600;}.skin4 button:focus {border-color: #8eff33;color: #8eff33;background-color: #2e6600;padding: 16px 16px;outline: 0;border-radius: 12px;}.skin4 button:focus:first-of-type {border-radius: 30px 12px 12px 30px;}.skin4 button:focus:last-of-type{border-radius: 12px 30px 30px 12px;}* {font-family: \"Muli\", sans-serif;}button::-moz-focus-inner{border: 0;}button{outline: 0;}body{background: #001519;height: 100vh;margin: 0;display: grid;place-items: center;justify-items: center;}.group-container{height: 100px;display: grid;place-items: center;justify-items: center;}.multi-button{font-size: 0;}button{font-weight: 800;background: transparent;min-width: 100px;text-transform: uppercase;border-radius: 8px;font-size: 15px;display: inline-block;padding: 10px 16px;margin: 0 3px 0 0;cursor: pointer;transition-duration: 0.1s;transition-timing-function: ease-in-out;}button:first-of-type{border-radius: 20px 8px 8px 20px;}button:last-of-type{border-radius: 8px 20px 20px 8px;}@media only screen and (max-width: 767px){button{min-width: 80px;}}.nohigh {user-select: none;-webkit-user-select: none;-moz-user-select: none;-ms-user-select: none;}.nohigh::selection{background: transparent;}</style><script>window.addEventListener(\"load\", () => {var all = document.getElementsByClassName(\"nohigh\");for (let a of all) { a.onselectstart = () => false; }});</script><div class=\"container nohigh\"><div class=\"group-container\"><div class=\"multi-button skin1\"><button onclick=\"window.location.href = \'/M\';\">LeftRot</button><button onclick=\"window.location.href = \'/N\';\">Forward</button><button onclick=\"window.location.href = \'/O\';\">RightRot</button></div></div><div class=\"group-container\"><div class=\"multi-button skin2\"><button onclick=\"window.location.href = \'/P\';\">&nbsp&nbsp&nbspLeft&nbsp&nbsp&nbsp&nbsp</button><button onclick=\"window.location.href = \'/Q\';\">&nbsp&nbsp&nbsp&nbspBrake&nbsp&nbsp&nbsp&nbsp</button><button onclick=\"window.location.href = \'/R\';\">&nbsp&nbspRight&nbsp&nbsp&nbsp&nbsp</button></div></div><div class=\"group-container\"><div class=\"multi-button skin3\"><button onclick=\"window.location.href = \'/S\';\">&nbspEmpty&nbsp&nbsp</button><button onclick=\"window.location.href = \'/T\';\">Backward</button><button onclick=\"window.location.href = \'/U\';\">&nbspEmpty&nbsp&nbsp&nbsp</button></div></div></div><!--<footer style=\"color:#FAF6ED; text-align: right; vertical-align: bottom; font-size: xx-small;\"  ><i>Chinmay Krishn Roy</i></footer>-->");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
        if (currentLine.endsWith("GET /M")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_DIAGONAL_135);
        }
        if (currentLine.endsWith("GET /N")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_STRAIGHT_FORWARD);
        }
        if (currentLine.endsWith("GET /O")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_DIAGONAL_45);
        }
        if (currentLine.endsWith("GET /P")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_SIDEWAYS_LEFT);
        }
        if (currentLine.endsWith("GET /Q")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_STOP);
        }
        if (currentLine.endsWith("GET /R")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_SIDEWAYS_RIGHT);
        }
        if (currentLine.endsWith("GET /S")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_ROTATE_COUNTERCLOCKWISE);
        }
        if (currentLine.endsWith("GET /T")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_STRAIGHT_BACKWARD);
        }
        if (currentLine.endsWith("GET /U")) {
          moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_ROTATE_CLOCKWISE);
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

void gimbleControl(int throttle, int throttleRange = 150) {
  throttle = map(throttle, -throttleRange, throttleRange, 0, 100);
  gimble.write(throttle);
}

void click(Button2& btn) {
  mode = 1;
  digitalWrite(LED_BUILTIN, 1);
  delay(100);
  digitalWrite(LED_BUILTIN, 0);
}

void longClick(Button2& btn) {
  stop();
}

void doubleClick(Button2& btn) {
  if (mode < 0 || mode >= TOTAL_MODE) mode = 0;
  mode += 1;
  digitalWrite(LED_BUILTIN, 1);
  delay(80);
  digitalWrite(LED_BUILTIN, 0);
  delay(80);
  digitalWrite(LED_BUILTIN, 1);
  delay(80);
  digitalWrite(LED_BUILTIN, 0);
}

void tripleClick(Button2& btn) {
  setup();
  digitalWrite(LED_BUILTIN, 1);
  delay(50);
  digitalWrite(LED_BUILTIN, 0);
  delay(50);
  digitalWrite(LED_BUILTIN, 1);
  delay(50);
  digitalWrite(LED_BUILTIN, 0);
  delay(50);
  digitalWrite(LED_BUILTIN, 1);
  delay(50);
  digitalWrite(LED_BUILTIN, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  motorsPinSet();
  Wire.begin();
  lox.begin();
  lox.startRangeContinuous();
  button.begin(BUTTON_PIN);
  button.setClickHandler(click);
  button.setDoubleClickHandler(doubleClick);
  button.setTripleClickHandler(tripleClick);
  button.setLongClickHandler(longClick);
  gimble.attach(2);
  huskylens.begin(Wire);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, passssword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  digitalWrite(LED_BUILTIN, 0);
  stop();
}

void loop() {
  unsigned long int now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;
  button.loop();
  if (lox.isRangeComplete()) {
    if (lox.readRange() < MIN_IN_MM) {
      digitalWrite(MF_AI2, LOW);
      digitalWrite(MF_BI2, LOW);
      digitalWrite(MR_AI2, LOW);
      digitalWrite(MR_BI2, LOW);
    }
  }
  if (mode == 0) {
    stop();
  } else if (mode == 1) {
    if (!huskylens.request()) {
      Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
      stop();
    } else if (!huskylens.isLearned()) {
      Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
      stop();
    } else if (!huskylens.available()) {
      Serial.println(F("No block or arrow appears on the screen!"));
      stop();
    } else {
      while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();
        updateControls(result, dt);
        Serial.println(String() + F("pitch=") + pitch + F(",yaw=") + yaw + F(",throttle=") + throttle);
        autoMove(pitch, yaw);
      }
    }
  } else if (mode == 2) {
    wifiMode();
  } else if (mode == 3) {
    if (!huskylens.request()) {
      Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
      wifiMode();
    } else if (!huskylens.isLearned()) {
      Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
      wifiMode();
    } else if (!huskylens.available()) {
      Serial.println(F("No block or arrow appears on the screen!"));
      wifiMode();
    } else {
      while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();
        updateControls(result, dt);
        Serial.println(String() + F("pitch=") + pitch + F(",yaw=") + yaw + F(",throttle=") + throttle);
        autoMove(pitch, yaw);
      }
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
