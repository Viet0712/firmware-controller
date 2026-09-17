// ESP32-S3 LiDAR + Encoder streaming over UDP at 500us intervals
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "esp_system.h"
#include <stdio.h>
#include <PID_v1.h>
#define RPWM_R 39  // Lam
#define LPWM_R 40  // Tim
#define R_EN_R 41  //VAng
#define L_EN_R 42  //luc

//motor right
#define RPWM_L 16  // Lam
#define LPWM_L 18  // Tim
#define R_EN_L 17  //VAng
#define L_EN_L 15  // luc
struct PIDOutput {
  int pwm;
  int direction;
};
volatile long encoderCountMotor1 = 0;
volatile long encoderCountMotor2 = 0;

volatile long encoderCountMotor1PID = 0;
volatile long encoderCountMotor2PID = 0;

volatile long encoderCountMotor1PID_prev = 0;
volatile long encoderCountMotor2PID_prev = 0;
int a = 5;
//A la Vàng B la Xanh
void IRAM_ATTR encoderISRL() {
  int b = digitalRead(10);
  if (b == HIGH) {
    encoderCountMotor2++;
    encoderCountMotor2PID++;
  } else {
    encoderCountMotor2--;
    encoderCountMotor2PID--;
  }
}

void IRAM_ATTR encoderISRR() {
  int b = digitalRead(11);
  if (b == HIGH) {
    encoderCountMotor1++;
    encoderCountMotor1PID++;
  } else {
    encoderCountMotor1--;
    encoderCountMotor1PID--;
  }
}


double Setpoint_L, Input_L, Output_L;
double Setpoint_R, Input_R, Output_R;
double Kp_L = 30.0, Ki_L = 10.0, Kd_L = 0.0;
double Kp_R = 30.0, Ki_R = 10.0, Kd_R = 0.0;

PID myPID_L(&Input_L, &Output_L, &Setpoint_L, Kp_L, Ki_L, Kd_L, DIRECT);


PID myPID_R(&Input_R, &Output_R, &Setpoint_R, Kp_R, Ki_R, Kd_R, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.print("start");
  // motor
  // -- encoder
  pinMode(9, INPUT_PULLUP);   //1 vang C2
  pinMode(10, INPUT_PULLUP);  //1 xanh la C1
  pinMode(11, INPUT_PULLUP);  //2 vang C2
  pinMode(12, INPUT_PULLUP);  //2 xanh la C1

  attachInterrupt(
    digitalPinToInterrupt(9),
    encoderISRL,
    RISING);
  attachInterrupt(
    digitalPinToInterrupt(12),
    encoderISRR,
    RISING);
  //--
  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);
  digitalWrite(R_EN_L, HIGH);  // Bật chiều phải
  digitalWrite(L_EN_L, HIGH);  // Bật chiều trái

  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);
  digitalWrite(R_EN_R, HIGH);  // Bật chiều phải
  digitalWrite(L_EN_R, HIGH);  // Bật chiều trái

  // Thiết lập PWM cho RPWM và LPWM
  ledcSetup(0, 1000, 8);
  ledcAttachPin(RPWM_L, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(LPWM_L, 1);


  ledcSetup(2, 1000, 8);
  ledcAttachPin(RPWM_R, 2);

  ledcSetup(3, 1000, 8);
  ledcAttachPin(LPWM_R, 3);
  Serial.println("setup done");

  myPID_L.SetMode(AUTOMATIC);
  myPID_L.SetOutputLimits(-255, 255); // nếu cần PWM 2 chiều

  myPID_R.SetMode(AUTOMATIC);
  myPID_R.SetOutputLimits(-255, 255); // nếu cần PWM 2 chiều
}
unsigned long lastEncoderTime = 0;
const unsigned long encoderInterval = 50;
float dt = encoderInterval / 1000.0f;  // giây
float v_l = 0;
float v_r = 0;
float w = 0;
float v = 0;
int CPR = 990;
float Radius_wheel = 0.065 / 2;
// Giả sử đã có encoder đo rps hiện tại để về sau nâng cấp PID
const float RPS_MAX = 1.0f;  // tối đa thực tế đo được
const int PWM_MAX = 200;
const int PWM_MIN = 20;  // deadband để bánh bắt đầu quay
float I_max = PWM_MAX * 0.35;

float KpL = 2.0f;
float KiL = 0.6f;
float KdL = 2.0f;

float KpR = 1.0f;
float KiR = 0.6f;
float KdR= 1.0f;
float error_prev_R = 0.0;
float error_prev_L = 0.0;
float IL = 0;
float IR = 0;
volatile long encoder_r;
volatile long encoder_l;
const float RPS_MIN_EFFECTIVE_FWD = 0.1f;
const float RPS_MIN_EFFECTIVE_REV = 0.1f;


float applyMinEffectiveRps(float cmd) {
  if (fabs(cmd) < 0.001f) return 0.0f;

  if (cmd > 0.0f && cmd < RPS_MIN_EFFECTIVE_FWD) {
    return RPS_MIN_EFFECTIVE_FWD;
  }

  if (cmd < 0.0f && fabs(cmd) < RPS_MIN_EFFECTIVE_REV) {
    return -RPS_MIN_EFFECTIVE_REV;
  }

  return cmd;
}
int mapRpsToPwm(float rps) {
  float mag = fabs(rps);
  mag = (mag > RPS_MAX) ? RPS_MAX : mag;
  int pwm = (mag <= 0.0001f) ? 0 : (int)(PWM_MIN + (PWM_MAX - PWM_MIN) * (mag / RPS_MAX));
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  return pwm;
}

float PIDL(float tg_rps, float real_rps) {
  float error = tg_rps - real_rps;
  float P = KpL * error;
  IL += KiL * error * dt;
  IL = constrain(IL, -I_max, I_max);
  float D = KdL * (error - error_prev_L) / dt;
  float u = P + IL + D;
  error_prev_L = error;
  return u;
}
float PIDR(float tg_rps, float real_rps) {
  float error = tg_rps - real_rps;
  float P = KpR * error;
  IR += KiR * error * dt;
  IR = constrain(IR, -I_max, I_max);
  float D = KdR * (error - error_prev_R) / dt;
  float u = P + IR + D;
  error_prev_R = error;
  return u;
}
float rpsl_md = 0.0;
float rpsr_md = 0.0;
void setWheelRps(float rpsL, float rpsR) {
 
  if (fabs(rpsL) < 0.001f) { IL = 0; error_prev_L = 0; rpsl_md = 0; }
  if (fabs(rpsR) < 0.001f) { IR = 0; error_prev_R = 0; rpsr_md = 0; }
  rpsL = constrain(rpsL, -RPS_MAX, RPS_MAX);
  rpsR = constrain(rpsR, -RPS_MAX, RPS_MAX);
  Setpoint_L = fabs(rpsL);
  Setpoint_R = fabs(rpsR);
  // Trái
  // int pL = mapRpsToPwm(rpsL);
  int pL = Output_L;
  if(pL > 0 && pL < PWM_MIN){
    pL = PWM_MIN;
  }
  // if(rpsL>0 && rpsR >0){
  //   pL*=1.18f;
  // }
  
  // if(rpsL<0 && rpsR <0){
  //   pL*=1.1f;
  // }



  // pL += rpsl_md;

  // Phải
  // int pR = mapRpsToPwm(rpsR);
  // pR += rpsr_md;

  // if (pL > 0 && rpsL > 0) {
  //   pL +=40;
  // }
  //  if (pL > 0 && rpsL < 0) {
  //   pL +=15;
  // }
  // int pR = pL;
  int pR = Output_R;
  if(pR > 0 && pR < PWM_MIN){
    pR = PWM_MIN;
  }
    

  if (rpsL < 0) {
    ledcWrite(0, pL);
    ledcWrite(1, 0);
  } else if (rpsL == 0) {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, pL);
  }
  if (rpsR < 0) {
    ledcWrite(3, pR);
    ledcWrite(2, 0);
  } else if (rpsR == 0) {
    ledcWrite(2, 0);
    ledcWrite(3, 0);
  } else {
    ledcWrite(3, 0);
    ledcWrite(2, pR);
  }
}
float v_left = 0.0;
float v_right = 0.0;
void loop() {
  if (millis() - lastEncoderTime >= encoderInterval) {
    static unsigned long last = 0;
    unsigned long now = millis();
    dt = (now - last) / 1000.0f;
    last = now;
    encoder_r = encoderCountMotor1PID - encoderCountMotor1PID_prev;
    encoder_l = encoderCountMotor2PID - encoderCountMotor2PID_prev;
    lastEncoderTime = millis();
    StaticJsonDocument<64> doc;
    doc["motor1"] = (float)encoder_r / CPR;
    doc["motor2"] = (float)encoder_l / CPR;
    serializeJson(doc, Serial);
    Serial.println();
    // Serial.println(String("Motor R: ") + String((float)encoder_r / CPR) + "\tMotor L: " + String((float)encoder_l / CPR));
    float real_rps_l = (encoder_l / (float)CPR) / dt;
    float real_rps_r = (encoder_r / (float)CPR) / dt;
    Input_L=fabs(real_rps_l);
    Input_R=fabs(real_rps_r);
    rpsl_md = PIDL(abs(v_left) , abs(real_rps_l));
    rpsr_md = PIDR(abs(v_right) , abs(real_rps_r));
    myPID_L.Compute();
    myPID_R.Compute();
    // Serial.println(String("Motor R_pid: ") + String(rpsr_md) + "\tMotor L_pid: " + String(rpsl_md));
    // encoderCountMotor1 = 0;
    // encoderCountMotor2 = 0;
    // float encoder_L_tg = getTargetEncoder(0.5);
    // float encoder_r_tg = getTargetEncoder(0.5);
    // // Serial.println(String("Motor R_tg: ") + String((float)encoder_r_tg) + "\tMotor L_tg: " + String((float)encoder_L_tg));
    // Serial.println(String("Motor R_tg: ") + String((float)encoder_r_tg / CPR) + "\tMotor L_tg: " + String((float)encoder_L_tg / CPR));
    // float v_actual_l = getReal_v(encoder_l);
    // float v_actual_r = getReal_v(encoder_r);
    // Serial.println(String("v R_: ") + String((float)v_actual_r) + "\tv L_: " + String((float)v_actual_l));
    encoderCountMotor1PID_prev = encoderCountMotor1PID;
    encoderCountMotor2PID_prev = encoderCountMotor2PID;
  }
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      StaticJsonDocument<256> doc;
      DeserializationError e = deserializeJson(doc, line);
      if (!e) {
        v_left = doc["v_left"] | 0.0;
        v_right = doc["v_right"] | 0.0;
        v_left = applyMinEffectiveRps(v_left);
        v_right = applyMinEffectiveRps(v_right);
        setWheelRps(v_left, v_right);
      }
      line = "";
    } else {
      line += c;
    }
  }
  // v_left = 0.1;
  // v_right = 0.1;
  // v_left = applyMinEffectiveRps(v_left);
  // v_right = applyMinEffectiveRps(v_right);
  // setWheelRps(v_left, v_right);

  // delay(1);
}
