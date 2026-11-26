/*
 * StringPlotter.ino  --  firmware for a 2-motor string (polargraph) plotter
 *
 * Hardware: Arduino UNO + CNC Shield V3 + A4988 or DRV8825 drivers
 *   LEFT  motor -> X stepper header  (STEP = D2, DIR = D5)
 *   RIGHT motor -> Y stepper header  (STEP = D3, DIR = D6)
 *   All drivers share ENABLE = D8 (active LOW = enabled)
 *   Z header unused.
 *   Optional pen-lift servo -> D12 ("SPIN EN" pin on the shield), see USE_SERVO.
 *
 * The PC (polargraph.py) does ALL the geometry: it converts SVG paths into
 * absolute step positions for both motors. This firmware only:
 *   - executes synchronized 2-axis moves with trapezoid acceleration
 *   - flips the optional pen servo
 *   - answers every command with "OK" on serial (that is the flow control)
 *
 * Serial protocol, 115200 baud, one command per line ('\n' terminated):
 *   M115                       -> replies "STRINGPLOT 1.0" then OK
 *   G92                        -> define current position as step 0 / 0
 *   G01 L<steps> R<steps> [F<steps_per_s>] [C<0|1>]
 *        absolute move. C1 = start and end the move at JOIN_FEED instead of
 *        a full stop (used for connected pen-down segments, smoother corners)
 *   JOG L<steps>  /  JOG R<steps>   relative test move, slow, for calibration
 *   M3 / M5                   -> pen down / pen up (needs USE_SERVO 1)
 *   M17 / M18                 -> enable / disable (release) motors
 *   M114                      -> replies "POS L<steps> R<steps>" then OK
 * Unknown command -> "ERR ..." (no OK).
 */

// ====================== USER SETTINGS ======================
#define USE_SERVO    1        // set to 1 when a pen-lift servo is installed
#define SERVO_PIN    12       // CNC shield "SPIN EN" pin (D12)
#define PEN_UP_ANGLE   95     // servo angle, pen lifted  (tune for your horn)
#define PEN_DOWN_ANGLE 30     // servo angle, pen touching board
#define PEN_DWELL_MS  200     // settle time after a pen up/down

#define INVERT_L 0            // set to 1 if the LEFT  motor winds the wrong way
#define INVERT_R 0            // set to 1 if the RIGHT motor winds the wrong way
// ============================================================

// CNC Shield V3 pinout (GRBL-style)
const uint8_t L_STEP_PIN = 2;
const uint8_t L_DIR_PIN  = 5;
const uint8_t R_STEP_PIN = 3;
const uint8_t R_DIR_PIN  = 6;
const uint8_t ENABLE_PIN = 8;

// Motion limits (steps, so they work at any microstepping)
const long  MAX_FEED  = 6000;     // max steps per second (dominant motor)
const long  MIN_FEED  = 50;
const float ACCEL     = 25000.0;  // steps per second^2
const float JOIN_FEED = 250.0;    // blend speed at C1 joints

long posL = 0, posR = 0;          // current absolute step positions
long feedDefault = 2000;

#if USE_SERVO
// Pen-lift servo driven WITHOUT any library: a burst of standard 50 Hz
// pulses (544..2400 us, same mapping as Servo.h) is enough to swing the
// horn; it then stays put by gear friction.
void servoMoveTo(uint8_t angle) {
  unsigned int us = 544U + ((unsigned int)angle * 1864U) / 180U;
  for (int i = 0; i < 40; i++) {            // ~0.8 s of pulses
    unsigned long t0 = micros();
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(us);
    digitalWrite(SERVO_PIN, LOW);
    while (micros() - t0 < 20000UL) { }     // rest of the 20 ms period
  }
}
#endif

// ------------------------------------------------------------
void setup() {
  pinMode(L_STEP_PIN, OUTPUT); pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_STEP_PIN, OUTPUT); pinMode(R_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(L_STEP_PIN, LOW);
  digitalWrite(R_STEP_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);          // LOW = drivers enabled (holding)

#if USE_SERVO
  pinMode(SERVO_PIN, OUTPUT);
  servoMoveTo(PEN_UP_ANGLE);
#endif

  Serial.begin(115200);
  delay(300);
  Serial.println(F("STRINGPLOT 1.0 READY"));
}

void loop() {
  static char lineBuf[72];
  static uint8_t lineLen = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ------------------------------------------------------------
inline int8_t sgn(long x) { return (x > 0) - (x < 0); }

void pulseStep(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(4);          // A4988 needs >=1us, DRV8825 >=1.9us
  digitalWrite(pin, LOW);
}

void enableMotors(bool on) {
  digitalWrite(ENABLE_PIN, on ? LOW : HIGH);
}

void penDown() {
#if USE_SERVO
  servoMoveTo(PEN_DOWN_ANGLE);
#endif
  delay(PEN_DWELL_MS);
}

void penUp() {
#if USE_SERVO
  servoMoveTo(PEN_UP_ANGLE);
#endif
  delay(PEN_DWELL_MS);
}

// Synchronized 2-axis move with trapezoidal velocity profile.
// Positive steps = wind string IN (pen pulled toward that motor).
void moveTo(long tL, long tR, long feed, bool continuous) {
  long dL = tL - posL;
  long dR = tR - posR;
  long n  = max(labs(dL), labs(dR));      // dominant axis step count
  if (n <= 0) return;

  feed = constrain(feed, MIN_FEED, MAX_FEED);
  float v0 = continuous ? JOIN_FEED : 0.0;   // entry speed
  float v1 = continuous ? JOIN_FEED : 0.0;   // exit speed
  float a  = ACCEL;

  // peak velocity if we never reach cruise (triangular profile)
  float vp2     = (2.0 * a * (float)n + v0 * v0 + v1 * v1) * 0.5;
  float vCruise = sqrt(vp2 > 1.0 ? vp2 : 1.0);
  if (vCruise > (float)feed) vCruise = (float)feed;

  float dAcc = (vCruise * vCruise - v0 * v0) / (2.0 * a);
  float dDec = (vCruise * vCruise - v1 * v1) / (2.0 * a);
  float nF   = (float)n;
  if (dAcc + dDec > nF) {                 // numeric safety clamp
    float k = nF / (dAcc + dDec);
    dAcc *= k; dDec *= k;
  }

  digitalWrite(L_DIR_PIN, (((dL > 0) ? 1 : 0) ^ INVERT_L) ? HIGH : LOW);
  digitalWrite(R_DIR_PIN, (((dR > 0) ? 1 : 0) ^ INVERT_R) ? HIGH : LOW);
  delayMicroseconds(10);

  long eL = 0, eR = 0;
  unsigned long t0  = micros();
  float tUs = 0.0;                        // cumulative target time (us)

  for (long k = 1; k <= n; k++) {
    float sm = (float)k - 0.5;            // midpoint of this step interval
    float v;
    if (sm <= dAcc) {
      v = sqrt(v0 * v0 + 2.0 * a * sm);
    } else if (sm > nF - dDec) {
      v = sqrt(v1 * v1 + 2.0 * a * (nF - sm));
    } else {
      v = vCruise;
    }
    if (v < 20.0) v = 20.0;               // never crawl (< 50us/step)
    tUs += 1.0e6 / v;
    unsigned long deadline = t0 + (unsigned long)tUs;

    // Bresenham: which motors step at this tick
    eL += dL;
    if (labs(eL) >= n) { eL -= (long)sgn(dL) * n; pulseStep(L_STEP_PIN); posL += sgn(dL); }
    eR += dR;
    if (labs(eR) >= n) { eR -= (long)sgn(dR) * n; pulseStep(R_STEP_PIN); posR += sgn(dR); }

    while ((long)(micros() - deadline) < 0) { /* busy wait */ }
  }
}

// ------------------------------------------------------------
void handleLine(char* s) {
  for (char* p = s; *p; ++p) *p = toupper(*p);

  char* tok[8];
  uint8_t nt = 0;
  char* p = strtok(s, " \t");
  while (p && nt < 8) { tok[nt++] = p; p = strtok(NULL, " \t"); }
  if (nt == 0) return;

  if      (!strcmp(tok[0], "M115")) { Serial.println(F("STRINGPLOT 1.0")); }
  else if (!strcmp(tok[0], "G92"))  { posL = 0; posR = 0; }
  else if (!strcmp(tok[0], "M114")) {
    Serial.print(F("POS L")); Serial.print(posL);
    Serial.print(F(" R"));    Serial.println(posR);
  }
  else if (!strcmp(tok[0], "M3"))   { penDown(); }
  else if (!strcmp(tok[0], "M5"))   { penUp(); }
  else if (!strcmp(tok[0], "M17"))  { enableMotors(true); }
  else if (!strcmp(tok[0], "M18"))  { enableMotors(false); }
  else if (!strcmp(tok[0], "G01") || !strcmp(tok[0], "G1")) {
    long tL = posL, tR = posR, f = feedDefault;
    bool cont = false;
    for (uint8_t i = 1; i < nt; i++) {
      if      (tok[i][0] == 'L') { tL = atol(tok[i] + 1); }
      else if (tok[i][0] == 'R') { tR = atol(tok[i] + 1); }
      else if (tok[i][0] == 'F') { f = atol(tok[i] + 1); feedDefault = f; }
      else if (tok[i][0] == 'C') { cont = (atol(tok[i] + 1) != 0); }
    }
    moveTo(tL, tR, f, cont);
  }
  else if (!strcmp(tok[0], "JOG") && nt >= 3) {
    char ax = tok[1][0];
    long st = atol(tok[2]);
    if      (ax == 'L') { moveTo(posL + st, posR, 1500, false); }
    else if (ax == 'R') { moveTo(posL, posR + st, 1500, false); }
    else { Serial.println(F("ERR axis")); return; }
  }
  else {
    Serial.println(F("ERR unknown command"));
    return;
  }
  Serial.println(F("OK"));
}
