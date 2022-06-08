#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>          // Wire.h Included
QMC5883LCompass compass;

// Motor Driver
#define ENA 5          // deklarasi pin ENA
#define IN1 4          // deklarasi pin IN1
#define IN2 7          // deklarasi pin IN2
#define IN3 2          // deklarasi pin IN3
#define IN4 3          // deklarasi pin IN4
#define ENB 6          // deklarasi pin ENB

// Shift Register for 5 IR Sensor
#define _Q7 8          // deklarasi pin Not Serial Input
#define  CP 9          // deklarasi pin Clock Pulse
#define _PL 10         // deklarasi pin Not Load / Shift
#define _CE 11         // deklarasi pin Not Clock Enable

// Ultrasonic Sensor 
#define ECHO 12        // deklarasi pin Echo
#define TRIG 13        // deklarasi pin Trigger

// I2C Connection and Other IR Sensor 
#define ir_pin_FL  A7  // deklarasi pin Front Left
#define ir_pin_BL  A6  // deklarasi pin Back Left
#define ir_pin_FR  A0  // deklarasi pin Front Right
#define ir_pin_BR  A1  // deklarasi pin Back Right
#define SDA        A4  // deklarasi pin SDA
#define SCL        A5  // deklarasi pin SCL
#define ir_pin_L   A2  // deklarasi pin Left
#define ir_pin_R   A3  // deklarasi pin Right

//---------------------------------

// Initial Values and Other Constants
#define init_v_L  180         // rightMotor mid. = 135 - 40(diff.) = 95 
#define init_v_R  180         // 255(max.) - 120(min.) / 2 = 135 + 15(offs.)
#define line_th   500         // Batas Analog mendeteksi line
#define line_th_D 300         // Batas Analog antara sensor gelap - sensor terang
#define Goal      25          // State Goal 
#define obs_dist  10          // Distance to Obstacle (cm)
#define motor_dir 1           // Default Direction of Motor

#define addr_MPU  0x0D        // Address IMU
#define addr_ESP  0x04        // Address ESP Cam
#define addr_RFID 0x09        // Address Nano Slave

#define C_lasterror 0.05
#define C_line 0.6
#define C_az 0.4

void setup() {
  Serial.begin(57600);               // Open serial communications and wait for port to open:9600
  while (!Serial) {
    ;                               // wait for serial port to connect. Needed for Native USB only
  }
//  Serial.println("Ready");
//         
//  Serial.println("CLEARDATA");      // Data Logging
//  Serial.println("LABEL,Date,Time, state, action, ret, lasterror, calculatedaz, calculatedline, error");
//  //Serial.println("LABEL,Date,Time, a, prev_a");
//  Serial.println("RESETTIMER");

  Wire.begin();             // Create Wire Object
  
  // Konfigurasi pin-pin sebagai Output  
  pinMode(ir_pin_L, INPUT);         // Line Sensor Depan        
  pinMode(ir_pin_R, INPUT);
  
  pinMode(ir_pin_FL, INPUT);        // Line Sensor Kiri
  pinMode(ir_pin_BL, INPUT);
  
  pinMode(ir_pin_FR, INPUT);        // Line Sensor Kanan
  pinMode(ir_pin_BR, INPUT);

  pinMode(TRIG, OUTPUT);            // Ultrasonic
  pinMode(ECHO, INPUT);

  pinMode(_Q7, INPUT);              // Shift Register
  pinMode(CP, OUTPUT);
  pinMode(_PL, OUTPUT);
  pinMode(_CE, OUTPUT);

  for (int i = 2; i < 8; i++) {     // Motor Driver
    pinMode(i, OUTPUT);
  }
  
  compass.init();                   // IMU Sensor 

}

// Global Variable
//uint8_t state;
int ir_val_L, ir_val_R;             // Sensors Reading
int ir_val_LL[2], ir_val_RR[2];
int prev_b;
byte ret = B00000;     

float error, errorLast, erroInte;   // PID Parameter
float err, ctrl_sig;
float Kp = 15;
float Ki = 0;
float Kd = 0;
float errorDiff;
float output;

int motor_v_L;                      // Motor Speed
int motor_v_R;

int state = 1, action = 2;              // Current State and Action

unsigned long oldTime;              // Time measure

float err_az;
int azSetPoint;
int prev_state = 1;
float lasterror, calculatedaz, calculatedline;
int abs_direction[4] = {25, 100, 160, 250};
int setPoint_direction;
int current_direction;
int mode = 0;

bool isNodeCondition1;
int read_pynq = 0;

int overflow = 0;
int readystate = 0;
char userInput[50];

void loop() {

  ///////////////////////////////////////////////////
  
  if (Serial.available() > 0) {
    
    String command;
    overflow = 1;

    for (int i = 0; i < 50; i++) {
        while(Serial.available() < 1) {}
        userInput[i] = Serial.read();
        if(userInput[i] == '\n' || userInput[i] == '\0') {
            overflow = 0;
            break;
        }
    }

    if(overflow == 1) {
      return;
    }
    command = String(userInput);

    if (command.startsWith("get")) {
        command.remove(0, 4);
        if(command.startsWith("internal")) {
            command.remove(0, 9);
            Serial.println("internal working");
        } else if(command.startsWith("state")) {
            command.remove(0, 6);
            Serial.println(state);
        } else if(command.startsWith("ready")) {
            command.remove(0, 6);
            Serial.println(readystate);
        }
    } else if (command.startsWith("set")) {
        command.remove(0, 4);
        if(command.startsWith("action")) {
            command.remove(0, 7);
            action = (int)command[0] - (int)'0';
            read_pynq = 1;
            
            Serial.print("set a");
            Serial.println(action);
        }
        if(command.startsWith("state")) {
            command.remove(0, 6);
            state = (int)command[0] - (int)'0';
            Serial.print("set s");
            Serial.println(state);
        }
    } else if (command.startsWith("g_status")) {
        Serial.println("[success] ");
    } else {
        Serial.flush();
    }
  }

  superLoop();
  delay(50);

}

void superLoop() {
  //  Default Condition Robot Always Moving
  //  readPYNQ(state,action);
    if (state != Goal) {
        if (mode == 0) {
            readystate = 1;
            if(read_pynq==0) return;
            read_pynq = 0;
            readystate = 0;
            rotate(action);
            mode = 1;
        } else if (mode == 1) {
            isNode();
            moveCar();
            moveCar2();
            
            ctrl_sig = calculatePID(err,err_az);        // Calculate PID
            motor_v_L = init_v_L + ctrl_sig;
            motor_v_R = init_v_R - ctrl_sig;
            motorWrite(motor_dir, motor_dir, motor_v_L, motor_v_R);
        }
    } else {
        // Serial.println("Robot telah mencapai finish!");
        motorWrite(0, 0, 0, 0);
        delay(10000);
    }
}

void moveCar2(){
  int az;
  compass.read();
  az = compass.getAzimuth();
  azSetPoint = abs_direction[action];
  err_az = (azSetPoint - az + 540) % 360 - 180;
}
  
void moveCar() {      
  readShiftReg(&ret);               // 5 IR front sensor
  
  switch (ret) {
    case B00000:
        err = 0;
        break;
    case B00001:
       err = -8;
       break;

    case B00011:
       err = -6;
       break;

    case B00010:
        err = -4;
       break;
       
    case B00110:
      err = -2;
      break;
      
    case B01110:
    case B00100:
      err = 0;
      break;
       
    case B01100:
      err = 2;
      break;

    case B01000:
      err = 4;
      break;

    case B11000:
      err = 6;
      break;

    case B10000:
      err = 8;
      break;
      
    default:
      err = err;
      break;
  }
}

int calculatePID(float err, float err_az) {
  unsigned long dt = calculateDeltaTime();
  lasterror = error * C_lasterror;
  calculatedaz = err_az/10 * C_az;
  calculatedline = err * C_line;
  
  error = lasterror + calculatedaz + calculatedline;   // Low Pass filter to smoothens Digital Reading
                                                       // err az positive on clockwise, err positive on B00001
  errorDiff = error - errorLast;
  erroInte = constrain(erroInte + error, -10, 10);
  output = Kp * error + Ki * erroInte  + Kd * errorDiff;
  errorLast = error;
  return output;
}

unsigned long calculateDeltaTime(){
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - oldTime;
  oldTime = currentTime;
  return deltaTime/100;
}

void readSensor(char pin1, char pin2, int*pin1Val, int*pin2Val) {
  *pin1Val = analogRead(pin1);
  *pin2Val = analogRead(pin2);
}

void readShiftReg(byte *ret){
  // Write pulse to _PL pin
  digitalWrite(_PL, LOW);
  delayMicroseconds(5);
  digitalWrite(_PL, HIGH);
  delayMicroseconds(5);
  
  // Get data from 74HC165
  digitalWrite(CP, HIGH);
  digitalWrite(_CE, LOW);
  byte  incoming = shiftIn(_Q7, CP, MSBFIRST);
  byte  mask = B00011111;
  digitalWrite(_CE, HIGH);
  *ret = (incoming & mask);
}

void motorWrite(bool en_forwardLeft, bool en_forwardRight, int motor_v_L, int motor_v_R) {
  digitalWrite(IN1, en_forwardLeft);
  digitalWrite(IN2, !en_forwardLeft);
  analogWrite(ENA, constrain(motor_v_L, 0, 250));     // Won't exceed 250 in speed

  digitalWrite(IN3, en_forwardRight);
  digitalWrite(IN4, !en_forwardRight);
  analogWrite(ENB, constrain(motor_v_R, 0, 250));
}

void rotate(int action) {
  bool isCW;
  compass.read();
  int vel, curdir = compass.getAzimuth();
  for (int i=0; i<3; i++){
    if (abs((abs_direction[action] - curdir + 540) % 360 - 180) < 30)
        return;
  }
  for (int i=0;i<3;i++){
      while (!isRotated(action, &isCW, &vel)) {      
        motorWrite(isCW, !isCW, vel, vel);
        delay(20);
      }
      motorWrite(0,0,0,0);
      delay(250);
  }
  motorWrite(0,0,0,0);
  delay(500);
}

bool isRotated (int action, bool *isCW, int *vel){
      compass.read();                
      int trgt = abs_direction[action];
      int azimuth = compass.getAzimuth();;
      int az_err = (trgt - azimuth + 540) % 360 - 180;
      *isCW = az_err > 0;
      
      if (abs(az_err) > 150)
          *vel = 180;
      else if (abs(az_err) > 120)
          *vel = 170;
      else if (abs(az_err) > 90)
          *vel = 150;
      else if (abs(az_err) > 60)
          *vel = 140;
      else if (abs(az_err) > 30)
          *vel = 130;
      else 
          *vel = 120;
      return abs(az_err) < 5;
}

int isNode() {
  bool isNode;
  
  // Check If It Meets Node
  if (digitalRead(ir_pin_R) == HIGH || digitalRead(ir_pin_L) == HIGH)
      isNode = true;
  else
      isNode = false;
  if (isNode) {
    motorWrite(0, 0, 0, 0);
    delay(250);
    int adjustMode = 0;             // 0: Too Front/Behind, 1: Too Right/Left
    
    while (!readRFID() || prev_state != state) {
      // Serial.println("Robot sedang membaca state!");
      // Serial.println();
      adjustCar(adjustMode);
      adjustMode = !adjustMode;
    }
    motorWrite(0, 0, 0, 0);
    delay(500);
  }
  return 1;
}

void adjustCar(int adjustMode) {
  int del, motor_dir_t;
  motorWrite(1,1, init_v_L, init_v_R);
  delay(50);
  motorWrite(0,0,0,0);
  readShiftReg(&ret);

  if(ret == B00100 || ret == B00000){
      if (digitalRead(ir_pin_FR)){
        motor_dir_t = 1;
      }
      else if (digitalRead(ir_pin_BR)){
        motor_dir_t = 0;  
      }
      else
        motor_dir_t = 1;
      motorWrite(motor_dir_t,motor_dir_t, init_v_L,init_v_R);
      delay(50);
//      motorWrite(0,0, init_v_L,init_v_R);
//      delay(50);
  } else {
      switch (ret) { 
    
            case B00001:
            case B00011:
            case B00111:
              del = 75;
              motor_dir_t = 0;            // Kanan Maju
              break;
                 
            case B00010:
            case B00110:
              del = 50;
              motor_dir_t = 0;            // Kanan Maju
              break;
        
            case B01000:
            case B01100:
              del = 50;
              motor_dir_t = 1;            // Kiri Maju
              break;
        
            case B10000:
            case B11000:
            case B11100:
              del = 75;
              motor_dir_t = 1;            // Kiri Maju
              break;
              
            default:
              del = 0;
              break;
        }
        
        if (del != 0) {
          motorWrite(motor_dir_t, !motor_dir_t, init_v_L, init_v_R);
          delay(del);
        }
  }
  motorWrite(0, 0, 0, 0);            // Stop Adjusting
}

bool isObstacle(){
  long duration; int distance;
  digitalWrite(TRIG, LOW);           // Clears the trigPin condition
  delayMicroseconds(2);
  
  digitalWrite(TRIG, HIGH);          // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  duration = pulseIn(ECHO, HIGH);    // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;   // Speed of sound wave divided by 2 (go and back)
  
//   Serial.print("Distance: ");        // Displays the distance on the Serial Monitor
//   Serial.print(distance);
//   Serial.println(" cm");

  if (distance < obs_dist) return HIGH; else return LOW;
}

//int readQR() {
//    int doneRead = 0, reading = 0, times_reading = 0;
//    
//    Serial.print("state_before: ");
//    Serial.println(state);
//    Serial.print("doneRead_before: ");
//    Serial.println(doneRead);
//      while (!doneRead && (times_reading < 4)) { // Limit QR Reading only 4 times
//        Wire.beginTransmission(addr_ESP);
//        Wire.write(byte(0x00));                  // Start with register[3].
//        Wire.endTransmission();
//
//        Wire.requestFrom(addr_ESP, 2);
//        delay(500);
//        
//        while(Wire.available()) {
//            doneRead = Wire.read();          // register[0]
//            reading = Wire.read();           // register[1]
//        }
//        times_reading++;
//    Serial.println("MASUK Kok");
//    Serial.print("reading: ");
//    Serial.println(reading);
//    Serial.print("doneRead: ");
//    Serial.println(doneRead);
//        }
//    state = reading;
//  if (  (state != prev_state) && (state > 0) && (state < 26) ) {
//  Serial.println("-----------------------------QR KEBACA------------");
//    delay(2000);}
//    return doneRead;
//}

int readRFID() {
    int doneRead = 0, reading = 0, times_reading = 0;
    
    while (times_reading < 4) { // Limit QR Reading only 4 times
      Wire.requestFrom(addr_RFID, 1);
      delay(50);
      
      while(Wire.available()) {
          reading = (int) Wire.read();           // register[1]
      }
      times_reading++;
//       Serial.println("MASUK Kok");
//       Serial.print("reading: ");
//       Serial.println(reading);
//       Serial.print("doneRead: ");
//       Serial.println(doneRead);
    }
    state = reading;
    if (  (state != prev_state) && (state > 0) && (state < 26) ) {
//       Serial.println("-----------------------------RFID KEBACA------------------------");
      doneRead = 1;
      // if (state == 6)
      //     action = 2;
      // else if (state == 11)
      //     action = 1;
      // else if (state == 12)
      //     action = 1;
      // else if (state == 13)
      //     action = 2;
      // else if (state == 18)
      //     action = 2;
      // else if (state == 23)
      //     action = 1;
      // else if (state == 24)
      //     action = 1;
      // else
      //     action = state % 4;
      mode = 0;
      prev_state = state;
    }
    return doneRead;
}

/*
// int readPYNQ(int state, int action){
//     int a = 0;
//     int s = 0;
//     int overflow = 0;
//     char userInput[50];
    
//     if (Serial.available() > 0) {
        
//         String command;
//         overflow = 1;

//         for (int i = 0; i < 50; i++) {
//             while(Serial.available() < 1) {}
//             userInput[i] = Serial.read();
//             if(userInput[i] == '\n' || userInput[i] == '\0') {
//                 overflow = 0;
//                 break;
//             }
//         }

//         if(overflow == 1) {
//             return;
//         }
//         command = String(userInput);

//         if (command.startsWith("get")) {
//             command.remove(0, 4);
//             if(command.startsWith("internal")) {
//                 command.remove(0, 9);
//                 Serial.println("internal working");
//             } else if(command.startsWith("state")) {
//                 command.remove(0, 6);
//                 Serial.println(s);
//             }
//         } else if (command.startsWith("set")) {
//             command.remove(0, 4);
//             if(command.startsWith("action")) {
//                 command.remove(0, 7);
//                 a = (int)command[0] + 1 - (int)'1';

//                 //////////////
//                 int nextstate, isValid;
//                 if (a == 0)
//                     nextstate = s - 5;
//                 else if (a == 1)
//                     nextstate = s + 1;
//                 else if (a == 2)
//                     nextstate = s + 5;
//                 else
//                     nextstate = s - 1;

//                 if (nextstate < 0)
//                     isValid = 0;
//                 else if (nextstate > 24)
//                     isValid = 0;
//                 else if ((s % 5 == 4) && (a == 1))
//                     isValid = 0;
//                 else if ((s % 5 == 0) && (a == 3))
//                     isValid = 0;
//                 else
//                     isValid = 1;

//                 if (isValid && s != 24) {
//                     s = nextstate;
//                 }
//                 /////////////
//                 Serial.print("set a");
//                 Serial.println(a);
//             }
//             if(command.startsWith("state")) {
//                 command.remove(0, 6);
//                 s = (int)command[0] + 1 - (int)'1';
//                 Serial.print("set s");
//                 Serial.println(s);
//             }
//         } else if (command.startsWith("g_status")) {
//             Serial.println("[success] ");
//         } else {
//             Serial.flush();
//         }
//         delay(500);
//     }
//     state = s;
//     action = a;
// }
*/

// void printSensor(int ir_val_L, int ir_val_R, int ir_val_LL0, 
//                  int ir_val_LL1, int ir_val_RR0, int ir_val_RR1){
//     Serial.print("Left Front = ");
//     Serial.println(ir_val_L);
//     Serial.print("Right Front = ");
//     Serial.println(ir_val_R);
//     Serial.print("Front Left = ");
//     Serial.println(ir_val_LL0);
//     Serial.print("Back Left = ");
//     Serial.println(ir_val_LL1);
//     Serial.print("Front Right = ");
//     Serial.println(ir_val_RR0);
//     Serial.print("Back Right = ");
//     Serial.println(ir_val_RR1);
//     Serial.println();
//     Serial.println();
// }
