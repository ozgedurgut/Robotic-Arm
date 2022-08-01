

#include <ServoTimer2.h>

#define j1_DIR_PIN          55 // X           BASE
#define j1_STEP_PIN         54
#define j1_ENABLE_PIN       38

#define j2_DIR_PIN          61 // Y           SHOULDER 
#define j2_STEP_PIN         60
#define j2_ENABLE_PIN       56

#define j3_DIR_PIN          48 // Z           SHOULDER
#define j3_STEP_PIN         46
#define j3_ENABLE_PIN       62

#define j4_DIR_PIN          28 //E0    A      ELBOW
#define j4_STEP_PIN         26
#define j4_ENABLE_PIN       24

#define j5_DIR_PIN          34 //E1    B      WRIST
#define j5_STEP_PIN         36
#define j5_ENABLE_PIN       30

#define servo_hand          4 //              HAND
#define servo_gripper       5 //              GRIPPER

// SWITCH PINS
#define base_switch         3
#define shoulder_switch     14
#define elbow_switch        18
#define wrist_switch        19

#define Joystick_PINX       A9    //RED
#define Joystick_PINY       A10    //WHITE

#define Joystick_PINX_hand      A11
#define Joystick_PINY_gripper   A12

#define Joystick_PINX_shoulder A15
#define Joystick_PINY_base A14

#define j1_STEP_HIGH  PORTF     |=   0b00000001;
#define j1_STEP_LOW   PORTF     &=  ~0b00000001;

#define j2_STEP_HIGH  PORTF     |=   0b01000000;
#define j2_STEP_LOW   PORTF     &=  ~0b01000000;

#define j3_STEP_HIGH  PORTL     |=   0b00001000;
#define j3_STEP_LOW   PORTL     &=  ~0b00001000;

#define j4_STEP_HIGH  PORTA     |=   0b00010000;
#define j4_STEP_LOW   PORTA     &=  ~0b00010000;

#define j5_STEP_HIGH  PORTC     |=   0b00000010;
#define j5_STEP_LOW   PORTC     &=  ~0b00000010;

#define TIMER1_INTERRUPTS_ON TIMSK1    |=    (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF TIMSK1   &=   ~(1 << OCIE1A);

String s_c1, s_c2, s_c3;
float  speedOrAccelValue;
int c1, c2, c3, c4, whichStepper, accelValue;

String String_oku;
int aci1, aci2, aci3, aci4, aci5, aci6, aci7;
String s_aci1, s_aci2, s_aci3, s_aci4, s_aci5, s_aci6;
float angle1, angle2, angle3, angle4, angle5, angle6;

int step1_aci;
int step2_aci;
int step3_aci;
int step4_aci;
int step5_aci;
int step6_aci;

float base_sabit = 35.555;          // 25600/360 ' dan gelen deger
float shoulder_sabit = 35.555;      // 12800/360
float elbow_sabit = 32.00;          // 11520/360
float wrist_sabit = 17.777;         // 2048/360
float hand_gripper_sabit = 10.00;   // (2400-600)/180 (0-180 derece arasi)

int initial_base;
int initial_shoulder;
int initial_elbow;
int initial_wrist;

int servo_pos1 = 2400;
int servo_pos = 600;

ServoTimer2 servo_hand_tmr2;        // 0-150
ServoTimer2 servo_gripper_tmr2;     // 50-150 servo dereceleri

int xPosition = 0;
int yPosition = 0;

int xPosition_hand = 0;
int yPosition_gripper = 0;
int xPosition_shoulder = 0;
int yPosition_base = 0;

struct motorInfo {

  float acceleration;
  volatile unsigned long minStepInterval;
  void (*dirFunc)(int);
  void (*stepFunc)();

  unsigned int firstStepInterval;  //c0
  long currentPositionStep;        //stepPosition

  volatile int dir;                             // current direction of movement
  volatile unsigned int totalSteps;             // number of steps requested for current movement
  volatile bool movementDone = false ;          // true if current movement completed
  volatile unsigned int rampUpStepCount;        // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
  volatile unsigned long estimatedStepToMaxSpeed;
  volatile unsigned long estimatedTimeForMove;  // estimated time required to complete movement
  volatile unsigned long rampUpStepTime;
  volatile float speedScale;

  volatile unsigned int n;          // num of index,,, index in acceleration curve
  volatile float d;                 // gecerli aralik uzunlugu
  volatile unsigned long di;        // gecerli aralik uzunlugunun kesildigi hali
  volatile unsigned int stepCount;  // num of steps completed in current movement
};

void j1Step() {
  j1_STEP_HIGH
  j1_STEP_LOW
}
void j1Direction(int dir) {
  digitalWrite(j1_DIR_PIN, dir);
}

void j2Step() {
  j2_STEP_HIGH
  j2_STEP_LOW
}
void j2Direction(int dir) {
  digitalWrite(j2_DIR_PIN, dir);
}

void j3Step() {
  j3_STEP_HIGH
  j3_STEP_LOW
}
void j3Direction(int dir) {
  digitalWrite(j3_DIR_PIN, dir);
}

void j4Step() {
  j4_STEP_HIGH
  j4_STEP_LOW
}
void j4Direction(int dir) {
  digitalWrite(j4_DIR_PIN, dir);
}

void j5Step() {
  j5_STEP_HIGH
  j5_STEP_LOW
}
void j5Direction(int dir) {
  digitalWrite(j5_DIR_PIN, dir);
}

#define NUM_MOTORS 6
volatile motorInfo motors[NUM_MOTORS];

void setup() {
  Serial.begin(9600);

  servo_hand_tmr2.attach(servo_hand);
  servo_gripper_tmr2.attach(servo_gripper);

  Serial.println("  MARK-I ROBOTIC ARM  ");

  pinMode(j1_STEP_PIN,    OUTPUT);
  pinMode(j1_DIR_PIN,     OUTPUT);
  pinMode(j1_ENABLE_PIN,  OUTPUT);

  pinMode(j2_STEP_PIN,    OUTPUT);
  pinMode(j2_DIR_PIN,     OUTPUT);
  pinMode(j2_ENABLE_PIN,  OUTPUT);

  pinMode(j3_STEP_PIN,    OUTPUT);
  pinMode(j3_DIR_PIN,     OUTPUT);
  pinMode(j3_ENABLE_PIN,  OUTPUT);

  pinMode(j4_STEP_PIN,    OUTPUT);
  pinMode(j4_DIR_PIN,     OUTPUT);
  pinMode(j4_ENABLE_PIN,  OUTPUT);

  pinMode(j5_STEP_PIN,    OUTPUT);
  pinMode(j5_DIR_PIN,     OUTPUT);
  pinMode(j5_ENABLE_PIN,  OUTPUT);

  pinMode(base_switch,    INPUT);
  pinMode(shoulder_switch, INPUT);
  pinMode(elbow_switch,   INPUT);
  pinMode(wrist_switch,   INPUT);

  pinMode(9,              OUTPUT); // fan
  digitalWrite(9,           HIGH);

  pinMode(Joystick_PINX, INPUT);
  pinMode(Joystick_PINY, INPUT);
  pinMode(Joystick_PINX_hand, INPUT);
  pinMode(Joystick_PINY_gripper, INPUT);
  pinMode(Joystick_PINX_shoulder, INPUT);
  pinMode(Joystick_PINY_base, INPUT);

  digitalWrite(j1_ENABLE_PIN, LOW);
  digitalWrite(j2_ENABLE_PIN, LOW);
  digitalWrite(j3_ENABLE_PIN, LOW);
  digitalWrite(j4_ENABLE_PIN, LOW);
  digitalWrite(j5_ENABLE_PIN, LOW);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                            // compare value
  TCCR1B |= (1 << WGM12);                  // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));   // 64 prescaler
  interrupts();

  motors[0].dirFunc = j1Direction;
  motors[0].stepFunc = j1Step;
  motors[0].acceleration = 1000;
  motors[0].minStepInterval = 100;

  motors[1].dirFunc = j2Direction;
  motors[1].stepFunc = j2Step;
  motors[1].acceleration = 1000;
  motors[1].minStepInterval = 350;

  motors[2].dirFunc = j3Direction;
  motors[2].stepFunc = j3Step;
  motors[2].acceleration = 1000;
  motors[2].minStepInterval = 350;

  motors[3].dirFunc = j4Direction;
  motors[3].stepFunc = j4Step;
  motors[3].acceleration = 500;
  motors[3].minStepInterval = 500;

  motors[4].dirFunc = j5Direction;
  motors[4].stepFunc = j5Step;
  motors[4].acceleration = 1000;
  motors[4].minStepInterval = 350;
}

void resetMotors (volatile motorInfo &mi) {
  mi.firstStepInterval = mi.acceleration;
  mi.d = mi.firstStepInterval;
  mi.di = mi.d;
  mi.stepCount = 0;
  mi.n = 0;
  mi.rampUpStepCount = 0;
  mi.movementDone = false;
  mi.speedScale = 1;

  float a = mi.minStepInterval / (float)mi.firstStepInterval;
  a *= 0.676;

  float k = ((a * a - 1) / (-2 * a));
  float n = k * k;

  mi.estimatedStepToMaxSpeed = n;
}

volatile byte remainingMotorFlag = 0;

float getDurationofAcceleration(volatile motorInfo &m, unsigned int numSteps)
{
  float d = m.firstStepInterval;
  float totalDuration = 0;
  for (unsigned int n = 1 ; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void prepareMovement (int whichMotor, long steps) {

  volatile motorInfo &mi = motors[whichMotor];
  mi.dirFunc(steps < 0 ? HIGH : LOW);
  mi.dir = steps > 0 ? 1 : -1;
  mi.totalSteps = abs(steps);
  resetMotors(mi);

  remainingMotorFlag |= (1 << whichMotor);
  unsigned long stepsAbsolute = abs(steps);

  if ((2 * mi.estimatedStepToMaxSpeed) < stepsAbsolute) {
    unsigned long stepsAtFullSpeed = stepsAbsolute - 2 * mi.estimatedStepToMaxSpeed;
    float accelDecelTime = getDurationofAcceleration(mi, mi.estimatedStepToMaxSpeed);
    mi.estimatedTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * mi.minStepInterval;
  }
  else {
    float accelDecelTime = getDurationofAcceleration(mi, stepsAbsolute / 2);
    mi.estimatedTimeForMove = 2 * accelDecelTime;
  }
}

volatile byte nextMotorFlag = 0;

void setNextInterruptInterval() {
  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (((1 << i) & remainingMotorFlag) && motors[i].di < mind) {
      mind = motors[i].di;
    }
  }
  nextMotorFlag = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].movementDone)
      movementComplete = false;
    if (((1 << i)& remainingMotorFlag) && motors[i].di == mind)
      nextMotorFlag |= (1 << i);
  }
  if (remainingMotorFlag == 0) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 65500;
  }
  OCR1A = mind;
}
ISR(TIMER1_COMPA_vect) // karsilastirma kesmesi
{
  unsigned int temporaryCounter = OCR1A; // OCR1A: karsilastirma registeri,, max miktar OCR1Ada tutulur
  OCR1A = 65500;

  for (int i = 0; i < NUM_MOTORS; i++) {
    if ( ! ((1 << i)& remainingMotorFlag))
      continue;

    if (! (nextMotorFlag & (1 << i))) {
      motors[i].di -= temporaryCounter;
      continue;
    }
    volatile motorInfo& m = motors[i];

    if (m.stepCount <= m.totalSteps) {
      m.stepFunc();
      m.stepCount++;
      m.currentPositionStep += m.dir;
      if ( m.stepCount >= m.totalSteps) {
        m.movementDone = true;
        remainingMotorFlag &= ~(1 << i);
      }
    }
    if (m.rampUpStepCount == 0) {
      m.n++;
      m.d = m.d - (2 * m.d) / (4 * m.n + 1);
      if (m.d <= m.minStepInterval) {
        m.d = m.minStepInterval;
        m.rampUpStepCount = m.stepCount;
      }
      if (m.stepCount >= m.totalSteps / 2) {
        m.rampUpStepCount = m.stepCount;
      }
      m.rampUpStepTime += m.d;
    }
    else if (m.stepCount >= m.totalSteps - m.rampUpStepCount) {
      m.d = (m.d * (4 * m.n + 1)) / (4 * m.n + 1 - 2);
      m.n--;
    }
    m.di = m.d * m.speedScale;
  }
  setNextInterruptInterval();
  TCNT1 = 0;
}

void runAndWait() {
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while ( remainingMotorFlag );
  remainingMotorFlag = 0;
  nextMotorFlag = 0;
}

void adjustSpeedScales() {
  float maxTime = 0;

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!((1 << i)& remainingMotorFlag))
      continue;
    if (motors[i].estimatedTimeForMove > maxTime)
      maxTime = motors[i].estimatedTimeForMove;
  }
  if (maxTime != 0) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!((1 << i)& remainingMotorFlag))
        continue;
      motors[i].speedScale = maxTime / motors[i].estimatedTimeForMove;
    }
  }
}

void base_shoulder_HOME()
{
  initial_shoulder = -1;
  initial_base = -1;
  while (digitalRead(shoulder_switch))
  {
    prepareMovement(0, initial_base);//3
    prepareMovement(1, initial_shoulder);//4
    prepareMovement(2, initial_shoulder);//5
    runAndWait();
    initial_base = initial_base - 1;
    initial_shoulder = initial_shoulder - 1;

    Serial.println("ARM SWITCH HIGH");
  }
  prepareMovement(0, 0);
  prepareMovement(1, 0);
  prepareMovement(2, 0);
  runAndWait();
  initial_base = -1;
  initial_shoulder = +1;

  while (digitalRead(base_switch))
  {
    prepareMovement(0, initial_base);
    prepareMovement(1, initial_shoulder);
    prepareMovement(2, initial_shoulder);
    runAndWait();
    initial_base = initial_base - 1;

    Serial.println("BASE SWITCH HIGH");
  }
  initial_shoulder = +1;

  prepareMovement(1, 0);
  prepareMovement(2, 0);
  runAndWait();
  while (!digitalRead(shoulder_switch))
  {
    initial_shoulder = initial_shoulder + 1;
    prepareMovement(1, initial_shoulder);
    prepareMovement(2, initial_shoulder);
    runAndWait();

    Serial.println("SHOULDER SWITCH LOW");
  }

  prepareMovement(0, 0);
  prepareMovement(1, 0);
  prepareMovement(2, 0);
  runAndWait();
  initial_base = 1;

  while (!digitalRead(base_switch))
  {
    initial_base = initial_base + 1;
    prepareMovement(0, initial_base);
    runAndWait();

    Serial.println("BASE SWITCH LOW");
  }

  prepareMovement(3, 0);
  runAndWait();

  Serial.println(" BASE-SHOULDER HOME HOME HOME");
  String_oku = "*45,45,45,1,80,150";
}

void wrist_elbow_HOME()
{
  initial_wrist = -1;
  initial_elbow = -1;

  while (digitalRead(wrist_switch))
  {
    prepareMovement(4, initial_wrist);
    prepareMovement(3, initial_elbow);
    runAndWait();
    initial_wrist = initial_wrist - 1;
    initial_elbow = initial_elbow - 1;

    Serial.println("WRIST SWITCH HIGH");
  }
  prepareMovement(4, 0);
  prepareMovement(3, 0);
  runAndWait();
  initial_wrist = +1;
  initial_elbow = -1;

  while (digitalRead(elbow_switch))
  {
    prepareMovement(4, initial_wrist);
    prepareMovement(3, initial_elbow);
    runAndWait();
    initial_elbow = initial_elbow - 1;
    //  initial_wrist = initial_wrist + 1 ;

    Serial.println("ELBOW SWITCH HIGH");
  }
  initial_wrist = +1;

  prepareMovement(4, 0);
  runAndWait();
  while (!digitalRead(wrist_switch))
  {
    initial_wrist = initial_wrist + 1;
    prepareMovement(4, initial_wrist);
    runAndWait();

    Serial.println("WRIST SWITCH LOW");
  }

  prepareMovement(4, 0);
  prepareMovement(3, 0);
  runAndWait();
  initial_elbow = 1;

  while (!digitalRead(elbow_switch))
  {
    initial_elbow = initial_elbow + 1;
    prepareMovement(3, initial_elbow);
    runAndWait();

    Serial.println("ELBOW SWITCH LOW");
  }

  prepareMovement(4, 3200);
  prepareMovement(3, 4320);
  runAndWait();

  Serial.println(" WRIST-ELBOW HOME HOME HOME");
}

void loop() {
  while (Serial.available()) {

    String_oku = Serial.readString();

    Serial.println();
    Serial.print("Alınan String : ");
    Serial.println(String_oku);
  }

  ////////////////////////////
  //       seriden AÇI      //                         *__,__,__,__,__,__
  ////////////////////////////

  if (String_oku.charAt(0) == '*')
  {
    aci1 = String_oku.indexOf("*");
    aci2 = String_oku.indexOf(",", aci1 + 1);
    aci3 = String_oku.indexOf(",", aci2 + 1);
    aci4 = String_oku.indexOf(",", aci3 + 1);
    aci5 = String_oku.indexOf(",", aci4 + 1);
    aci6 = String_oku.indexOf(",", aci5 + 1);
    aci7 = String_oku.indexOf(",", aci6 + 1);

    s_aci1 = String_oku.substring(aci1 + 1, aci2);
    s_aci2 = String_oku.substring(aci2 + 1, aci3);
    s_aci3 = String_oku.substring(aci3 + 1, aci4);
    s_aci4 = String_oku.substring(aci4 + 1, aci5);
    s_aci5 = String_oku.substring(aci5 + 1, aci6);
    s_aci6 = String_oku.substring(aci6 + 1, aci7);

    angle1 = s_aci1.toFloat();
    angle2 = s_aci2.toFloat();
    angle3 = s_aci3.toFloat();
    angle4 = s_aci4.toFloat();
    angle5 = s_aci5.toFloat();
    angle6 = s_aci6.toFloat();

    step1_aci = (angle1 * base_sabit) + 0.5;        // eksen1        BASE
    step2_aci = (angle2 * shoulder_sabit) + 0.5;    // eksen2        SHOULDER
    step3_aci = (angle3 * elbow_sabit) + 0.5;       // eksen3        ELBOW
    step4_aci = (angle4 * wrist_sabit) + 0.5;       // eksen4        WRIST
    step5_aci = (angle5 * hand_gripper_sabit);      // servo_hand    HAND
    step6_aci = (angle6 * hand_gripper_sabit);      // servo_gripper GRIPPER
    step5_aci = step5_aci + 600;                    // servotimer2 0 derece başlaması için
    step6_aci = step6_aci + 600;                    // servotimer2 0 derece başlaması için

    Serial.print(step1_aci);    Serial.print("/");
    Serial.print(step2_aci);    Serial.print("/");
    Serial.print(step3_aci);    Serial.print("/");
    Serial.print(step4_aci);    Serial.print("/");
    Serial.print(step5_aci);    Serial.print("/");
    Serial.print(step6_aci);
    Serial.println();

    prepareMovement(0, step1_aci); // x soketi  - eksen1 - base
    prepareMovement(1, step2_aci); // y soketi  - eksen2 - arm-shoulder
    prepareMovement(2, step2_aci); // z soketi  - eksen2 - arm-shoulder
    prepareMovement(3, step3_aci); // e0 soketi - eksen3 - arm - elbow
    prepareMovement(4, step4_aci); // e1 soketi - eksen4 - arm - wrist

    servo_hand_tmr2.write(step5_aci);
    servo_gripper_tmr2.write(step6_aci);
    runAndWait();

    Serial.println("Movement Completed!");

    String_oku = "";


  }
  else if (String_oku.charAt(0) == 'H')
  {
    Serial.println("H received.");
    Serial.println("GO HOME !");

    wrist_elbow_HOME();
    base_shoulder_HOME();
  }


  ////////////////////////////
  //    seriden adımlama    //
  ////////////////////////////

  else if (String_oku.charAt(0) == 'j'  ) {
    while (1) {

      char c = Serial.read();

      if (c == '1') {
        prepareMovement(3, 10);
        runAndWait();
        Serial.println("motor elbow CW 1 step");
      }
      else if (c == '2') {
        prepareMovement(3, -10);
        runAndWait();
        Serial.println("motor elbow CCW 1 step");
      }
      else if (c == '3') {
        prepareMovement(4, 10);
        runAndWait();
        Serial.println("motor wrist CW 1 step");
      }
      else if (c == '4') {
        prepareMovement(4, -10);
        Serial.println("motor wrist CCW 1 step");
        runAndWait();
      }
      else if (c == '5') {
        prepareMovement(0, 10);
        runAndWait();
        Serial.println("motor base CW 1 step");
      }
      else if (c == '6') {
        prepareMovement(0, -10);
        runAndWait();
        Serial.println("motor base CCW 1 step");
      }
      else if ( c == '7') {
        prepareMovement(1, 10 );
        prepareMovement(2, 10 );
        runAndWait();
        Serial.println("motor shoulders CW 1 step");
      }
      else if ( c == '8') {
        prepareMovement(1, -10 );
        prepareMovement(2, -10 );
        runAndWait();
        Serial.println("motor shoulders CCW 1 step");
      }
      else if ( c == '9') {
        break;
      }
      String_oku = "";
    }


    //////////////////////////
    // HIZ VE IVME AYARLAMA //
    //////////////////////////

  }  else if (String_oku.charAt(0) == '#') {

    Serial.println(" ACCELERATION AND SPEED CHANGE MODE ACTIVATED " );

    c2 = String_oku.indexOf(":", 0);
    c3 = String_oku.indexOf(",", c2 + 1);
    c4 = String_oku.indexOf(",", c3 + 1);

    s_c1 = String_oku.substring(0 + 1, c2);
    s_c2 = String_oku.substring(c2 + 1, c3);
    s_c3 = String_oku.substring(c3 + 1, c4);

    whichStepper = s_c2.toInt();
    speedOrAccelValue = s_c3.toFloat();

    if (String_oku.substring(1, 9) == "setSpeed") {
      Serial.println("SPEED CHANGE MODE ACTIVATED");
      motors[whichStepper].minStepInterval = speedOrAccelValue;

      Serial.print("Motor: ");
      Serial.print(whichStepper);
      Serial.print("  |  ");
      Serial.print("Speed: ");
      Serial.println(speedOrAccelValue);
    }

    else if (String_oku.substring(1, 9) == "setAccel") {
      Serial.println("ACCELERATION CHANGE MODE ACTIVATED");
      accelValue = speedOrAccelValue * 100;
      motors[whichStepper].acceleration = accelValue;

      Serial.print("Motor: ");
      Serial.print(whichStepper );
      Serial.print("  |  ");
      Serial.print("Acceleration: ");
      Serial.println(speedOrAccelValue);

      prepareMovement(whichStepper, 500); //for test
      runAndWait();
    }
    String_oku = "";
  }

  ////////////////////////////////
  //       JOYSTICK MODE        //
  ////////////////////////////////

  // TÜM EKLEMLER JOYSTICKE BAĞLANDI

  else if (String_oku.charAt(0) == 'T') {

    xPosition = analogRead(Joystick_PINX);
    yPosition = analogRead(Joystick_PINY);
    xPosition_hand = analogRead(Joystick_PINX_hand);
    yPosition_gripper = analogRead(Joystick_PINY_gripper);
    xPosition_shoulder = analogRead(Joystick_PINX_shoulder);
    yPosition_base = analogRead(Joystick_PINY_base);


    while (xPosition_hand < 10 && servo_pos < 2400) {
      servo_pos += 10;
      servo_hand_tmr2.write(servo_pos);
      Serial.println(servo_pos);
      delay(15);
      if (250 < xPosition_hand < 750) {
        break;
      }
    }
    
    while (xPosition_hand > 1000 && servo_pos > 600 ) {
      servo_pos -= 10;
      servo_hand_tmr2.write(servo_pos);
      Serial.println(servo_pos);
      delay(15);
      if (250 < xPosition_hand < 750) {
        break;
      }
    }
    
    while (yPosition_gripper < 10 &&  servo_pos1 < 2400) {
      servo_pos1 += 10;
      servo_gripper_tmr2.write(servo_pos1);
      Serial.println(servo_pos1);
      delay(15);
      if (250 < yPosition_gripper < 750) {
        break;
      }
    }
    
    while (yPosition_gripper > 1000 &&  servo_pos1 > 600) {
      servo_pos1 -= 10;
      servo_gripper_tmr2.write(servo_pos1);
      Serial.println(servo_pos1);
      delay(15);
      if (250 < yPosition_gripper < 750) {
        break;
      }
    }
    
    while (xPosition < 10) {
      prepareMovement(3, 90);    // ELBOW
      runAndWait();
      if (250 < xPosition  < 750) {
        break;
      }
    }
    
    while (xPosition > 1000) {
      prepareMovement(3, -90);    // ELBOW
      runAndWait();
      if (250 < xPosition < 750) {
        break;
      }
    }
    
    while (yPosition < 10) {
      prepareMovement(4, -300);   // WRIST
      runAndWait();
      if ( 250 < yPosition < 750) {
        break;
      }
    }
    
    while (yPosition > 1000) {
      prepareMovement(4, 300);   // WRIST
      runAndWait();
      if (250 < yPosition < 750) {
        break;
      }
    }

    while (xPosition_shoulder < 10) {
      prepareMovement(1, 10 );
      prepareMovement(2, 10 );
      runAndWait();
      if (250 < xPosition_shoulder < 750) {
        break;
      }
    }

    while (xPosition_shoulder > 1000) {
      prepareMovement(1, -10 );
      prepareMovement(2, -10 );
      runAndWait();
      if (250 < xPosition_shoulder < 750) {
        break;
      }
    }
    
    while (yPosition_base < 10) {
      prepareMovement(0, 10 );
      runAndWait();
      if (250 < yPosition_base < 750) {
        break;
      }
    }
    
    while (yPosition_base > 1000) {
      prepareMovement(0, -10 );
      runAndWait();
      if (250 < yPosition_base < 750) {
        break;
      }
    }
  }
}
