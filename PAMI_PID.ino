// ================== 2 MOTEURS + 2 ENCODEURS (Nano ATmega328P) ==================

// ----- ENCODEURS -----
// A doit etre sur D2 ou D3 (interruptions). B peut etre sur n'importe quelle pin digitale.
const int encA = 2; // A (INT0)
const int encB = 7; // B
const int encA_D = 3; // Droit A (INT1)
const int encB_D = 8; // Droit B

// FIT0450 : 11 impulsions / tour moteur (sur A en RISING)
const int PPR_MOTEUR = 11;

const unsigned long periodeMesure = 100; // ms
const int MAX_RPM = 60;

volatile long count = 0;
volatile long countD = 0;
unsigned long tPrev = 0;
float rpm = 0.0f;
float rpmD = 0.0f;
bool measurePpr = false;
long pprStart = 0;

// ----- CONSIGNE FIXE -----
const int FIXED_RPM = 60;

// ----- DRIVER MOTEUR (H-bridge classique) -----
const int pwmPin = 9;  // PWM 
const int in1 = 10;  // Direction 
const int in2 = 6; // Direction 

const int pwmD = 5; // PWM droit
const int in1D = 11; // Direction droit (inverse in1D/in2D)
const int in2D = 4; // Direction droit (inverse in1D/in2D)

int pwmCmd = 200; // 0..255
int pwmCmdD = 200; // 0..255
int sens = 1;    // 1 = avant, -1 = arriere
const int sensGlobal = -1; // inverse les deux moteurs si l'avant part en arriere
const int sensDroit = 1; // sens normal pour le moteur droit (pins deja inverses)

// ----- PID VITESSE -----
bool usePid = true;
float targetRpm = 0.0f;
float targetRpmD = 0.0f;
// Trim vitesse: >1.0 = droite plus rapide, <1.0 = droite plus lente
// Trim vitesse gauche: <1.0 = gauche plus lente, >1.0 = gauche plus rapide
float trimG = 1.00f;

// Ajuste ces gains selon ton montage (separes pour chaque moteur)
// Gains du moteur selon la vitesse (gain scheduling)
const int RPM_LOW = 20;
const int RPM_MID = 40;

float Kp_low = 0.024f, Ki_low = 0.47f, Kd_low = 0.0f;
float Kp_mid = 0.015f, Ki_mid = 0.74f, Kd_mid = 0.0f;
float Kp_high = 0.04f, Ki_high = 1.15f, Kd_high = 0.0f;

float Kp = Kp_low;
float Ki = Ki_low;
const float Kd = 0.0f;

// Gains moteur droit (copie des gains gauche pour viser la meme vitesse)
float KpD_low = 0.022f, KiD_low = 0.45f, KdD_low = 0.0f;
float KpD_mid = 0.025f, KiD_mid = 0.78f, KdD_mid = 0.0f;
float KpD_high = 0.04f, KiD_high = 1.15f, KdD_high = 0.0f;

float KpD = KpD_low;
float KiD = KiD_low;
const float KdD = 0.0f;

float iTerm = 0.0f;
float iTermD = 0.0f;
float prevErr = 0.0f;
float prevErrD = 0.0f;

const float iClamp = 200.0f;
const float KP_STEP = 0.1f;
const float KI_STEP = 0.05f;

enum TuneTarget { TUNE_BOTH, TUNE_LEFT, TUNE_RIGHT };
TuneTarget tuneTarget = TUNE_BOTH;

const int CMD_BUF_LEN = 32;
char cmdBuf[CMD_BUF_LEN];
byte cmdLen = 0;

void adjustGainsLeft(float targetRpm, float dKp, float dKi) {
 if (targetRpm <= RPM_LOW) {
  Kp_low = max(0.0f, Kp_low + dKp);
  Ki_low = max(0.0f, Ki_low + dKi);
 } else if (targetRpm <= RPM_MID) {
  Kp_mid = max(0.0f, Kp_mid + dKp);
  Ki_mid = max(0.0f, Ki_mid + dKi);
 } else {
  Kp_high = max(0.0f, Kp_high + dKp);
  Ki_high = max(0.0f, Ki_high + dKi);
 }
}

void adjustGainsRight(float targetRpm, float dKp, float dKi) {
 if (targetRpm <= RPM_LOW) {
  KpD_low = max(0.0f, KpD_low + dKp);
  KiD_low = max(0.0f, KiD_low + dKi);
 } else if (targetRpm <= RPM_MID) {
  KpD_mid = max(0.0f, KpD_mid + dKp);
  KiD_mid = max(0.0f, KiD_mid + dKi);
 } else {
  KpD_high = max(0.0f, KpD_high + dKp);
  KiD_high = max(0.0f, KiD_high + dKi);
 }
}

int parseIntFrom(const char *s) {
 while (*s && (*s < '0' || *s > '9')) s++;
 return atoi(s);
}

void applyGainDelta(float dKp, float dKi) {
 if (tuneTarget == TUNE_LEFT || tuneTarget == TUNE_BOTH) {
  adjustGainsLeft(targetRpm, dKp, dKi);
  selectGains(targetRpm);
 }
 if (tuneTarget == TUNE_RIGHT || tuneTarget == TUNE_BOTH) {
  adjustGainsRight(targetRpmD, dKp, dKi);
  selectGainsD(targetRpmD);
 }
}

void processCommand(const char *cmd) {
 if (!cmd || !cmd[0]) return;

 // single-letter commands
 if (cmd[1] == '\0') {
  char c = cmd[0];
  if (c == 'f') { sens = 1; setMotor(pwmCmd, sens * sensGlobal); setMotorD(pwmCmdD, sens * sensDroit * sensGlobal); }
  if (c == 'r') { sens = -1; setMotor(pwmCmd, sens * sensGlobal); setMotorD(pwmCmdD, sens * sensDroit * sensGlobal); }
  if (c == 's') { stopMotors(); }
  if (c == 'p') { usePid = !usePid; }
  if (c == 'm') {
   if (!measurePpr) {
    noInterrupts();
    pprStart = count;
    interrupts();
    measurePpr = true;
   } else {
    long pprEndG;
    noInterrupts();
    pprEndG = count;
    interrupts();
    long pprWheel = pprEndG - pprStart;
    measurePpr = false;
   }
  }
  if (c == 'L') { tuneTarget = TUNE_LEFT; }
  if (c == 'R') { tuneTarget = TUNE_RIGHT; }
  if (c == 'B') { tuneTarget = TUNE_BOTH; }

  // compat: old +/- and i/o
  if (c == '+') { applyGainDelta(KP_STEP, 0.0f); }
  if (c == '-') { applyGainDelta(-KP_STEP, 0.0f); }
  if (c == 'i') { applyGainDelta(0.0f, KI_STEP); }
  if (c == 'o') { applyGainDelta(0.0f, -KI_STEP); }
  return;
 }

 // vXXX, gXXX, dXXX
 if (cmd[0] == 'v') {
  int val = parseIntFrom(cmd + 1);
  targetRpm = constrain(val, 0, MAX_RPM);
  targetRpmD = constrain(val, 0, MAX_RPM);
  usePid = true;
  return;
 }
 if (cmd[0] == 'g') {
  int val = parseIntFrom(cmd + 1);
  pwmCmd = constrain(val, 0, 255);
  usePid = false;
  setMotor(pwmCmd, sens * sensGlobal);
  setMotorD(pwmCmdD, sens * sensDroit * sensGlobal);
  return;
 }
 if (cmd[0] == 'd') {
  int val = parseIntFrom(cmd + 1);
  pwmCmdD = constrain(val, 0, 255);
  usePid = false;
  setMotorD(pwmCmdD, sens * sensDroit * sensGlobal);
  return;
 }

 // kp++++ / kp---- / ki+++ / ki---
 if (cmd[0] == 'k' && (cmd[1] == 'p' || cmd[1] == 'i')) {
  int plusCount = 0;
  int minusCount = 0;
  for (int i = 2; cmd[i] != '\0'; i++) {
   if (cmd[i] == '+') plusCount++;
   else if (cmd[i] == '-') minusCount++;
   else return;
  }
  int count = plusCount - minusCount;
  if (count == 0) return;
  if (cmd[1] == 'p') applyGainDelta(KP_STEP * (float)count, 0.0f);
  if (cmd[1] == 'i') applyGainDelta(0.0f, KI_STEP * (float)count);
  return;
 }
}

// ------------------ INTERRUPTIONS ENCODEURS ------------------
void isr() {
 if (digitalRead(encB) == HIGH) count++;
 else count--;
}

void isrD() {
 if (digitalRead(encB_D) == HIGH) countD++;
 else countD--;
}

// ------------------ FONCTIONS MOTEUR ------------------
void setMotor(int pwmVal, int direction) {
 pwmVal = constrain(pwmVal, 0, 255);
 // Trim applique directement sur la commande PWM gauche
 pwmVal = (int)(pwmVal * trimG + 0.5f);
 pwmVal = constrain(pwmVal, 0, 255);
 pwmVal = 255 - pwmVal;
 direction = -direction; // inversion logiciel globale
 if (direction >= 0) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
 } else {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 }
 analogWrite(pwmPin, pwmVal);
}

void setMotorD(int pwmVal, int direction) {
 pwmVal = constrain(pwmVal, 0, 255);
 direction = -direction; // inversion logiciel globale
 if (direction >= 0) {
  digitalWrite(in1D, HIGH);
  digitalWrite(in2D, LOW);
 } else {
  digitalWrite(in1D, LOW);
  digitalWrite(in2D, HIGH);
 }
 analogWrite(pwmD, pwmVal);
}

void stopMotors() {
 analogWrite(pwmPin, 0);
 digitalWrite(in1, LOW);
 digitalWrite(in2, LOW);
}

int pidStep(float targetRpm, float measRpm, float dtSec, float &iTerm, float &prevErr,
      float Kp, float Ki, float Kd) {
 float err = targetRpm - measRpm;
 iTerm += err * dtSec;
 iTerm = constrain(iTerm, -iClamp, iClamp);
 float dErr = (dtSec > 0.0f) ? (err - prevErr) / dtSec : 0.0f;
 prevErr = err;
 float out = (Kp * err) + (Ki * iTerm) + (Kd * dErr);
 int pwm = (int)constrain(out, 0.0f, 255.0f);
 return pwm;
}

void selectGains(float targetRpm) {
 if (targetRpm <= RPM_LOW) {
  Kp = Kp_low; Ki = Ki_low;
 } else if (targetRpm <= RPM_MID) {
  Kp = Kp_mid; Ki = Ki_mid;
 } else {
  Kp = Kp_high; Ki = Ki_high;
 }
}
void selectGainsD(float targetRpm) {
 if (targetRpm <= RPM_LOW) {
  KpD = KpD_low; KiD = KiD_low;
 } else if (targetRpm <= RPM_MID) {
  KpD = KpD_mid; KiD = KiD_mid;
 } else {
  KpD = KpD_high; KiD = KiD_high;
 }
}

// ------------------ SETUP ------------------
void setup() {
 Serial.begin(9600);

 pinMode(encA, INPUT_PULLUP);
 pinMode(encB, INPUT_PULLUP);
 pinMode(encA_D, INPUT_PULLUP);
 pinMode(encB_D, INPUT_PULLUP);

 pinMode(pwmPin, OUTPUT);
 pinMode(in1, OUTPUT);
 pinMode(in2, OUTPUT);
 pinMode(pwmD, OUTPUT);
 pinMode(in1D, OUTPUT);
 pinMode(in2D, OUTPUT);

 attachInterrupt(digitalPinToInterrupt(encA), isr, RISING);
attachInterrupt(digitalPinToInterrupt(encA_D), isrD, RISING);

 tPrev = millis();

 setMotor(pwmCmd, sens * sensGlobal);
setMotorD(pwmCmdD, sens * sensDroit * sensGlobal);

 targetRpm = FIXED_RPM;
 targetRpmD = FIXED_RPM;
 usePid = true;

 Serial.println("Cmd: vXXX (rpm), gXXX/dXXX PWM, f/r/s, p PID on/off, +/- Kp, i/o Ki, L/R/B target");
}

// ------------------ LOOP ------------------
void loop() {
 if (Serial.available()) {
  char c = Serial.read();

  if (c == 'f') { sens = 1; setMotor(pwmCmd, sens * sensGlobal); setMotorD(pwmCmdD, sens * sensDroit * sensGlobal); }
if (c == 'r') { sens = -1; setMotor(pwmCmd, sens * sensGlobal); setMotorD(pwmCmdD, sens * sensDroit * sensGlobal); }
if (c == 's') { stopMotors(); }
  if (c == 'p') { usePid = !usePid; }
  if (c == 'm') {
   if (!measurePpr) {
    noInterrupts();
    pprStart = count;
    interrupts();
    measurePpr = true;
    //Serial.println("PPR: demarre. Tourne la roue 1 tour puis appuie encore sur 'm'.");
   } else {
    long pprEndG;
    noInterrupts();
    pprEndG = count;
    interrupts();
    long pprWheel = pprEndG - pprStart;
    measurePpr = false;
    //Serial.print("PPR roue (impulsions/1 tour) = ");
    //Serial.println(pprWheel);
   }
  }
  if (c == 'L') { tuneTarget = TUNE_LEFT; }
  if (c == 'R') { tuneTarget = TUNE_RIGHT; }
  if (c == 'B') { tuneTarget = TUNE_BOTH; }

  if (c == '+') {
   if (tuneTarget == TUNE_LEFT || tuneTarget == TUNE_BOTH) {
    adjustGainsLeft(targetRpm, KP_STEP, 0.0f);
    selectGains(targetRpm);
   }
   if (tuneTarget == TUNE_RIGHT || tuneTarget == TUNE_BOTH) {
    adjustGainsRight(targetRpmD, KP_STEP, 0.0f);
    selectGainsD(targetRpmD);
   }
  }
  if (c == '-') {
   if (tuneTarget == TUNE_LEFT || tuneTarget == TUNE_BOTH) {
    adjustGainsLeft(targetRpm, -KP_STEP, 0.0f);
    selectGains(targetRpm);
   }
   if (tuneTarget == TUNE_RIGHT || tuneTarget == TUNE_BOTH) {
    adjustGainsRight(targetRpmD, -KP_STEP, 0.0f);
    selectGainsD(targetRpmD);
   }
  }
  if (c == 'i') {
   if (tuneTarget == TUNE_LEFT || tuneTarget == TUNE_BOTH) {
    adjustGainsLeft(targetRpm, 0.0f, KI_STEP);
    selectGains(targetRpm);
   }
   if (tuneTarget == TUNE_RIGHT || tuneTarget == TUNE_BOTH) {
    adjustGainsRight(targetRpmD, 0.0f, KI_STEP);
    selectGainsD(targetRpmD);
   }
  }
  if (c == 'o') {
   if (tuneTarget == TUNE_LEFT || tuneTarget == TUNE_BOTH) {
    adjustGainsLeft(targetRpm, 0.0f, -KI_STEP);
    selectGains(targetRpm);
   }
   if (tuneTarget == TUNE_RIGHT || tuneTarget == TUNE_BOTH) {
    adjustGainsRight(targetRpmD, 0.0f, -KI_STEP);
    selectGainsD(targetRpmD);
   }
  }

  if (c == 'g') { // ex: g150 (PWM manuel)
   int val = Serial.parseInt();
   pwmCmd = constrain(val, 0, 255);
   usePid = false;
   setMotor(pwmCmd, sens * sensGlobal);
setMotorD(pwmCmdD, sens * sensDroit * sensGlobal);
  }
  if (c == 'd') { // ex: d150 (PWM manuel)
 int val = Serial.parseInt();
 pwmCmdD = constrain(val, 0, 255);
 usePid = false;
 setMotorD(pwmCmdD, sens * sensDroit * sensGlobal);
}
if (c == 'v') { // ex: v120 (rpm consigne pour les deux)
   int val = Serial.parseInt();
   targetRpm = constrain(val, 0, MAX_RPM);
   targetRpmD = constrain(val, 0, MAX_RPM);
   usePid = true;
  }
 }

 unsigned long t = millis();
 if (t - tPrev >= periodeMesure) {
  noInterrupts();
  long imp = count; count = 0;
long impD = countD; countD = 0;
  interrupts();

  float tours = (float)imp / PPR_MOTEUR;
float toursD = (float)impD / PPR_MOTEUR;
  float dtMs = (float)(t - tPrev);
  float dtSec = dtMs / 1000.0f;
  rpm = (tours * 600.0f) / dtMs;
  rpmD = (toursD * 600.0f) / dtMs;

  if (usePid) {
   float meas = fabs(rpm);
float measD = fabs(rpmD);
   selectGains(targetRpm);
   selectGainsD(targetRpmD);
   pwmCmd = pidStep(targetRpm, meas, dtSec, iTerm, prevErr, Kp, Ki, Kd);
   pwmCmdD = pidStep(targetRpmD, measD, dtSec, iTermD, prevErrD, KpD, KiD, KdD);
   setMotor(pwmCmd, sens * sensGlobal);
   setMotorD(pwmCmdD, sens * sensDroit * sensGlobal);
  }

  // Serial Plotter: courbes rpm + consignes + impulsions + dt
  Serial.print("rpmG:");
  Serial.print(rpm);
  Serial.print(" rpmD:");
  Serial.print(fabs(rpmD));
  Serial.print(" consG:");
  Serial.print(targetRpm);
  Serial.print(" consD:");
  Serial.print(targetRpmD);

  Serial.print(" dtMs:");
  Serial.println(dtMs);

  tPrev = t;
 }
}
