
//Pins for QTI connections on board
#define lineSensor1 47  // left
#define lineSensor2 51  // old midleft- now middle
//#define lineSensor3 53 // old midright- not used
#define lineSensor3 52  // right
#define QTI_THRESHOLD 250

// Define pins for built-in RGB LED, and other status LEDs
#define redpin 45
#define greenpin 46
#define bluepin 44
#define TxPin 14
#define ColdTempPin 4
#define TxDisplayPin 2
#define RxDisplayPin 3

#include <Servo.h>             // Include servo library
#include <Wire.h>              // I2C library, required for MLX90614
#include <SparkFunMLX90614.h>  //Click here to get the library: http://librarymanager/All#Qwiic_IR_Thermometer by SparkFun
#include <SoftwareSerial.h>    // for LCD

SoftwareSerial mySerial = SoftwareSerial(255, TxPin);  // set up serial for LCD Serial.

Servo servoLeft;  // Declare left and right servos
Servo servoRight;

const int MAX_FORWARD_RIGHTSERVO = 1700;
const int MAX_FORWARD_LEFTSERVO = 1370;
const int MID_LEFT = 1500;
const int MID_RIGHT = 1500;
const int VEER_OFFSET = 40;

int hash_idx = 0;
int rgb_cycle[][3] = { { 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 0, 255, 255 } };

int hoth_pos = 0;
IRTherm therm;  // Create an IRTherm object to interact with throughout

int idc_state[5] = { -1, -1, -1, -1, -1 };

int vent_hash = 0;
bool teamsReady = false;
bool vent_running = false;
bool char_sent = false;

void moveUntilNthMark(int n) {
  //  delay(ventMoveTime);
  // move forward until middle sensor detects vent line
  int qti1 = rcTime(lineSensor1);
  int qti2 = rcTime(lineSensor2);
  int qti3 = rcTime(lineSensor3);
  int state1 = qtiToDigital(qti1);
  int state2 = qtiToDigital(qti2);
  int state3 = qtiToDigital(qti3);
  int stateOverall = 4 * state1 + 2 * state2 + 1 * state3;
  while (stateOverall != 0) {
    updateMotors(1300, 1700);
    delay(50);
    qti1 = rcTime(lineSensor1);
    state1 = qtiToDigital(qti1);
    qti2 = rcTime(lineSensor2);
    state2 = qtiToDigital(qti2);
    qti3 = rcTime(lineSensor3);
    state3 = qtiToDigital(qti3);
    stateOverall = 4 * state1 + 2 * state2 + 1 * state3;
  }
  updateMotors(1500, 1500);
  delay(500);

  // move forward slightly
  updateMotors(1300, 1700);
  delay(400);

  // read qti state again
  qti1 = rcTime(lineSensor1);
  state1 = qtiToDigital(qti1);
  qti2 = rcTime(lineSensor2);
  state2 = qtiToDigital(qti2);
  qti3 = rcTime(lineSensor3);
  state3 = qtiToDigital(qti3);
  stateOverall = 4 * state1 + 2 * state2 + 1 * state3;
  // move foward until vent shaft reached
  while (stateOverall != 0) {
    updateMotors(1300, 1700);
    delay(50);
    qti1 = rcTime(lineSensor1);
    state1 = qtiToDigital(qti1);
    qti2 = rcTime(lineSensor2);
    state2 = qtiToDigital(qti2);
    qti3 = rcTime(lineSensor3);
    state3 = qtiToDigital(qti3);
    stateOverall = 4 * state1 + 2 * state2 + 1 * state3;
  }
  updateMotors(1500, 1500);
  delay(1000);

  // move forward slightly
  updateMotors(1300, 1700);
  delay(300);

  // reread sensor 2
  qti2 = rcTime(lineSensor2);
  state2 = qtiToDigital(qti2);
  while (state2 == 1) {
    updateMotors(1700, 1700);
    delay(50);
    qti2 = rcTime(lineSensor2);
    state2 = qtiToDigital(qti2);
  }
  updateMotors(1500, 1500);

  delay(500);

  int ventState = 0;
  while (ventState < n) {
    int qti1 = rcTime(lineSensor1);
    int qti2 = rcTime(lineSensor2);
    int qti3 = rcTime(lineSensor3);
    delay(50);

    // convert qti readings into 0 or 1 (dark or light)
    int state1 = qtiToDigital(qti1);
    int state2 = qtiToDigital(qti2);
    int state3 = qtiToDigital(qti3);
    int stateOverall = 4 * state1 + 2 * state2 + 1 * state3;

    if (hash_idx > 5) {
      stateOverall = 8;
    }

    switch (stateOverall) {
      case 0:  // DDD, Hashmark
        if (ventState < n - 1) {
          updateMotors(1300, 1700);
          delay(200);
        }
        ventState = (ventState + 1);
        break;
      case 1:  // DDL, Slight left turn
        //      updateMotors(1550, 1300);
        updateMotors(1300, 1550);
        break;
      case 2:  // DLD, Move forward
        //      updateMotors(1700, 1300);
        updateMotors(1300, 1700);
        break;
      case 3:  // DLL, Left turn
        //      updateMotors(1450, 1300);
        updateMotors(1300, 1450);
        break;
      case 4:  // LDD, Slight right turn
        //      updateMotors(1700, 1450);
        updateMotors(1450, 1700);
        break;
      case 5:  // LDL, Move forward
        //      updateMotors(1700, 1300);
        updateMotors(1300, 1700);
        break;
      case 6:  // LLD, Right turn
        //      updateMotors(1700, 1550);
        updateMotors(1550, 1700);
        break;
      case 7:  // LLL
        //      updateMotors(1500, 1500);
        updateMotors(1500, 1500);
        break;
      default:
        updateMotors(1500, 1500);
        break;
    }
  }

  updateMotors(1500, 1500);
  // Serial2.print((char) (codes[botIndex] + 1));
}


void nav_full_forward() {
  servoLeft.writeMicroseconds(MAX_FORWARD_LEFTSERVO);    // Left wheel counterclockwise
  servoRight.writeMicroseconds(MAX_FORWARD_RIGHTSERVO);  // Right wheel clockwise
}

void nav_veer_left() {
  //servoLeft.writeMicroseconds(MAX_FORWARD_LEFTSERVO + 50);         // Left wheel counterclockwise
  //servoRight.writeMicroseconds(MAX_FORWARD_RIGHTSERVO);        // Right wheel clockwise
  servoLeft.writeMicroseconds(MID_LEFT);                  // Left wheel counterclockwise
  servoRight.writeMicroseconds(MID_RIGHT + VEER_OFFSET);  // Right wheel clockwise
  //delay(50);
  servoLeft.writeMicroseconds(MID_LEFT - VEER_OFFSET);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(MID_RIGHT);              // Right wheel clockwise
}

void nav_veer_right() {
  //servoLeft.writeMicroseconds(MAX_FORWARD_LEFTSERVO);         // Left wheel counterclockwise
  //servoRight.writeMicroseconds(MAX_FORWARD_RIGHTSERVO - 50);        // Right wheel clockwise
  servoLeft.writeMicroseconds(MID_LEFT - VEER_OFFSET);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(MID_RIGHT);              // Right wheel clockwise
  //delay(50);
  servoLeft.writeMicroseconds(MID_LEFT);                  // Left wheel counterclockwise
  servoRight.writeMicroseconds(MID_RIGHT + VEER_OFFSET);  // Right wheel clockwise
}

void nav_corner_left() {
  servoLeft.writeMicroseconds(MAX_FORWARD_LEFTSERVO);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(MID_RIGHT);             // Right wheel clockwise
}

void nav_corner_right() {
  servoLeft.writeMicroseconds(MID_LEFT);                 // Left wheel counterclockwise
  servoRight.writeMicroseconds(MAX_FORWARD_RIGHTSERVO);  // Right wheel clockwise
}

void hash_pause() {
  if (Serial.available()) {
    for (int i = 0; i < 5; i++) {
      idc_state[i] = 5;
    }
  }

  hash_idx++;

  if (!teamsReady) {
    // pause for one second
    servoLeft.writeMicroseconds(MID_LEFT);    // Left wheel counterclockwise
    servoRight.writeMicroseconds(MID_RIGHT);  // Right wheel clockwise

    // rgb cycle for hash change visibility.
    set_RGBi(rgb_cycle[hash_idx][0], rgb_cycle[hash_idx][1], rgb_cycle[hash_idx][2]);
    delay(100);

    //old loc for hidx++
    //Serial.println("rgb signal");
    if (hash_idx > 5) {
      // Display Hash Mark for our line for 2 seconds.
      //hash_idx = 0;
      int ascii = 3 * 5 + (hoth_pos - 1) + 65;
      Serial2.print((char)ascii);  // send to xbee

      mySerial.write(12);
      delay(10);
      servoLeft.detach();
      servoRight.detach();
      //servoLeft.writeMicroseconds(MID_LEFT);    // Left wheel counterclockwise
      //servoRight.writeMicroseconds(MID_RIGHT);  // Right wheel clockwise

      //mySerial.print("" + String(therm.object(), 2));
      mySerial.print("Hoth found at   hash: ");  // write to LCD
      mySerial.print(hoth_pos);
      delay(20);

      // Common Line Code:
      while (!teamsReady) {  // Actions for sending/recieving data on the common line. Execute till all teams are ready.
        if (!char_sent) {
          Serial2.print((char)ascii);  // send to xbee
          char_sent = true;
        }
        digitalWrite(TxDisplayPin, HIGH);
        delay(200);
        digitalWrite(TxDisplayPin, LOW);

        if (Serial2.available()) {
          char incoming = Serial2.read();
          int state = ((int)incoming - 65) % 5;
          int bot = ((int)incoming - 65) / 5;

          //Serial.print("bot: ");
          //Serial.println(bot);
          //Serial.print("state: ");
          //Serial.println(state);

          idc_state[bot] = state;

          if (incoming != "\n") {
            digitalWrite(RxDisplayPin, HIGH);
            delay(200);
            digitalWrite(RxDisplayPin, LOW);
          }
        } else {
          digitalWrite(RxDisplayPin, LOW);
        }

        teamsReady = true;

        mySerial.write(12);  // clear LCD
        delay(10);
        mySerial.print("IDC Code:       ");
        for (int i = 0; i < 5; i++) {
          //Serial.print(idc_state[i]);
          mySerial.print(idc_state[i] + 1);
          delay(10);
          if (i != 4) {
            mySerial.print(",");
            delay(10);
          }

          // teams are not ready if any one of the states is -1.
          if (idc_state[i] == -1) {
            teamsReady = false;
          }
        }
        // LCD update delay:
        delay(20);
      }
      //set_RGBi(0, 0, 0);
    }

    // measure temp of object.
    if (therm.read()) {
      /*Serial.print("Object: " + String(therm.object(), 2));
        Serial.println("F");
        Serial.print("Ambient: " + String(therm.ambient(), 2));
        Serial.println("F");
        Serial.println();*/
    }

    //mySerial.write(12);
    //mySerial.print("" + String(therm.object(), 2));

    if (therm.object() < 70) {
      hoth_pos = hash_idx;
      Serial.println("Hoth found at");
      Serial.print("hash mark: ");
      Serial.println(hoth_pos);
      idc_state[3] = hoth_pos - 1;
      digitalWrite(ColdTempPin, HIGH);
    } else {
      digitalWrite(ColdTempPin, LOW);
    }


    //manually move forward (but not on fifth hash):
    if (hash_idx < 5) {
      servoLeft.writeMicroseconds(MAX_FORWARD_LEFTSERVO);    // Left wheel counterclockwise
      servoRight.writeMicroseconds(MAX_FORWARD_RIGHTSERVO);  // Right wheel clockwise
      delay(200);
    }
  }

  // Vent Run Code:
  else {
    int incoming;
    vent_running = true;
    //forward for set time, from hash 5 past common line to vent line.
    if (Serial2.available()) {
      incoming = (int)Serial2.read() - 48;
      Serial.println(incoming);
    }
    
    if (incoming == hoth_pos || hoth_pos == 1) {
      // now we can reattach the servos.
      if (!servoLeft.attached()) {
        servoLeft.attach(11);
        servoRight.attach(12);
      }

      updateMotors(1370, 1700);
      delay(1600);
      updateMotors(1500, 1500);
      delay(500);
      updateMotors(1700, 1700);
      delay(500);

      /*int qti2 = rcTime(lineSensor2);
        int state2 = qtiToDigital(qti2);
        while (state2 == 1) {
        updateMotors(1700, 1700);
        delay(50);
        qti2 = rcTime(lineSensor2);
        state2 = qtiToDigital(qti2);
        }
        updateMotors(1500, 1500);
        */

      vent_run();
    }
  }
}

void vent_run() {
  hash_idx = 5;
  while (true) {
    int incoming;
    //forward for set time, from hash 5 past common line to vent line.
    if (Serial2.available()) {
      incoming = (int)Serial2.read() - 47;
      Serial.println(incoming);
    }

    Serial.print(hash_idx);
    Serial.println(hoth_pos);


    //qti read
    int qti1 = rcTime(lineSensor1);
    int qti2 = rcTime(lineSensor2);
    int qti3 = rcTime(lineSensor3);
    delay(50);
    //  Serial.println(qti1);
    //  Serial.println(qti2);
    //  Serial.println(qti3);
    // convert qti readings into 0 or 1 (dark or light)
    int state1 = qtiToDigital(qti1);
    int state2 = qtiToDigital(qti2);
    int state3 = qtiToDigital(qti3);
    int stateOverall = 4 * state1 + 2 * state2 + 1 * state3;


    //nav_3sensor(stateOverall);
    switch (stateOverall) {
      case 0:  // DDD, Hashmark
        //hash_pause();
        updateMotors(1500, 1500);
        delay(100);
        hash_idx++;
        updateMotors(1370, 1700);
        delay(150);

        // when we reach proper line, stop and send char for next bot to go.
        if (hash_idx >= 11 - hoth_pos) {
          // stop motors
          updateMotors(1500, 1500);

          // detach motors
          if (servoLeft.attached()) {
            servoLeft.detach();
            servoRight.detach();
          }

          // print continuously to xbee (but don't spam)
          while (true) {
            // our call sign as bot 4 is 3, so the fifth bot is 4.
            Serial2.print(hoth_pos + 1);  // send to xbee
            delay(8000);
          }
        }

        break;
      case 1:  // DDL, Slight left turn
        //      updateMotors(1550, 1300);
        updateMotors(1370, 1550);
        break;
      case 2:  // DLD, Move forward
        //      updateMotors(1700, 1300);
        updateMotors(1370, 1700);
        break;
      case 3:  // DLL, Left turn
        //      updateMotors(1450, 1300);
        updateMotors(1370, 1450);
        break;
      case 4:  // LDD, Slight right turn
        //      updateMotors(1700, 1450);
        updateMotors(1450, 1700);
        break;
      case 5:  // LDL, Move forward
        //      updateMotors(1700, 1300);
        updateMotors(1370, 1700);
        break;
      case 6:  // LLD, Right turn
        //      updateMotors(1700, 1550);
        updateMotors(1550, 1700);
        break;
      case 7:  // LLL
        //      updateMotors(1500, 1500);
        updateMotors(1370, 1700);
        break;
      default:
        break;
    }
  }
  //freeze on the hoth_pos vent_run mark.
  if (hash_idx == 7 + hoth_pos) {
    updateMotors(1500, 1500);
    while (true) {
    }
  }
}



void set_RGB(int r, int g, int b) {
  // Set RGB LED pins based on low=bright (default)
  analogWrite(redpin, r);
  analogWrite(greenpin, g);
  analogWrite(bluepin, b);
}

void set_RGBi(int r, int g, int b) {
  // Set RGB LED pins based on high=bright
  set_RGB(255 - r, 255 - g, 255 - b);
}

void updateMotors(int leftFreq, int rightFreq) {
  servoLeft.writeMicroseconds(leftFreq);
  servoRight.writeMicroseconds(rightFreq);
}

void nav_3sensor(int state) {
  switch (state) {
    case 0:  // DDD, Hashmark
      hash_pause();
      break;
    case 1:  // DDL, Slight left turn
      //      updateMotors(1550, 1300);
      updateMotors(1370, 1550);
      break;
    case 2:  // DLD, Move forward
      //      updateMotors(1700, 1300);
      updateMotors(1370, 1700);
      break;
    case 3:  // DLL, Left turn
      //      updateMotors(1450, 1300);
      updateMotors(1370, 1450);
      break;
    case 4:  // LDD, Slight right turn
      //      updateMotors(1700, 1450);
      updateMotors(1450, 1700);
      break;
    case 5:  // LDL, Move forward
      //      updateMotors(1700, 1300);
      updateMotors(1370, 1700);
      break;
    case 6:  // LLD, Right turn
      //      updateMotors(1700, 1550);
      updateMotors(1550, 1700);
      break;
    case 7:  // LLL
      //      updateMotors(1500, 1500);
      updateMotors(1370, 1700);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);   //start the serial monitor so we can view the output
  Serial2.begin(9600);  //start the serial monitor that prints to the XBee
  mySerial.begin(9600);
  Wire.begin();  //Joing I2C bus
  mySerial.write(12);

  if (therm.begin() == false) {  // Initialize thermal IR sensor
    Serial.println("Qwiic IR thermometer did not acknowledge! Freezing!");
    while (1)
      ;
  }
  Serial.println("Qwiic IR Thermometer did acknowledge.");

  therm.setUnit(TEMP_F);  // Set the library's units to Farenheit (sic)
  // Alternatively, TEMP_F can be replaced with TEMP_C for Celsius or
  // TEMP_K for Kelvin.


  servoLeft.attach(11);   // Attach left signal to P13
  servoRight.attach(12);  // Attach right signal to P12

  // Set pin modes
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(ColdTempPin, OUTPUT);
  pinMode(TxDisplayPin, OUTPUT);
  pinMode(RxDisplayPin, OUTPUT);

  set_RGBi(0, 0, 0);
}

boolean qtiToDigital(int qti) {
  if (qti > 105) {
    return 0;  // dark
  }
  return 1;  // light
}

void loop() {
  //qti read
  int qti1 = rcTime(lineSensor1);
  int qti2 = rcTime(lineSensor2);
  int qti3 = rcTime(lineSensor3);
  delay(50);
  //  Serial.println(qti1);
  //  Serial.println(qti2);
  //  Serial.println(qti3);
  // convert qti readings into 0 or 1 (dark or light)
  int state1 = qtiToDigital(qti1);
  int state2 = qtiToDigital(qti2);
  int state3 = qtiToDigital(qti3);
  int stateOverall = 4 * state1 + 2 * state2 + 1 * state3;
  //Serial.println(stateOverall);

  //navigation
  //nav_4sensor(state);
  nav_3sensor(stateOverall);
}

//Defines funtion 'rcTime' to read value from QTI sensor
// From Ch. 6 Activity 2 of Robotics with the BOE Shield for Arduino
long rcTime(int pin) {
  pinMode(pin, OUTPUT);     // Sets pin as OUTPUT
  digitalWrite(pin, HIGH);  // Pin HIGH
  delay(1);                 // Waits for 1 millisecond
  pinMode(pin, INPUT);      // Sets pin as INPUT
  digitalWrite(pin, LOW);   // Pin LOW
  long time = micros();     // Tracks starting time
  while (digitalRead(pin))
    ;                      // Loops while voltage is high
  time = micros() - time;  // Calculate decay time
  return time;             // Return decay time
}
