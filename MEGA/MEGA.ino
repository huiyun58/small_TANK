#include "wheel_drv.h"
#include <avr/io.h>`

//!!!
//void readCmd_wheel_angularVel() {
//  if (Serial.available() >= 6) {
//    char rT = (char)Serial.read(); // read target speed from mega
//    if (rT == '{') {               // read "{"
//      char commandArray[5];
//      // mySerial.readBytes(commandArray,4);
//      Serial.readBytes(commandArray, 5);    //read "rHrLlHlL}"
//      byte rH = commandArray[0];
//      byte rL = commandArray[1];
//      byte lH = commandArray[2];
//      byte lL = commandArray[3];
//      char rP = commandArray[4];
//      if (rP == '}') {
//        target_receive_R = (rH << 8) + rL;        //combine two bytes to increase resolution
//        target_receive_L = (lH << 8) + lL;
//        // convert received 16 bit integer to actual speed
//        //MaxSpeed depends on motors' performance
//        omega_target_R = double(target_receive_R * (double(MaxSpeed) / 32767));
//        omega_target_L = double(target_receive_L * (double(MaxSpeed) / 32767));
//        lastRecvMilli = millis();
////        Serial.print("omega_target_R:");
////        Serial.println(omega_target_R);
////        Serial.print("omega_target_L:");
////        Serial.println(omega_target_L);
////        Serial.println(lH, BIN);
////        Serial.println(lL, BIN);
//      }
//    }
//  }
//}

//!!!
void readCMD() {
  int L_index, R_index, sum_index, end_index, unlock_index;
  float Lspeed, Rspeed, sum;
    while (!recieveComplete) {
    if (Serial.available() > 0) {
      rec = Serial.read();
      recbuf += rec;
      } else {
      recieveComplete = true;
    }
  }
  // stop when connection lost
  if ((millis() - lastRecvMilli) > 500) {
    omega_target_L = 0;
    omega_target_R = 0;
  }
  // format: L|Lspeed|R|Rspeed|sum|speedsum|end
  L_index = recbuf.indexOf('L');
  R_index = recbuf.indexOf('R');
  sum_index = recbuf.indexOf("sum");
  end_index = recbuf.indexOf("end");

  if (L_index != -1 && recbuf.indexOf('R') != -1) {
    Lspeed = (recbuf.substring(L_index + 1, R_index)).toFloat();
    Rspeed = (recbuf.substring(R_index + 1, sum_index)).toFloat();
    sum = (recbuf.substring(sum_index + 3, end_index)).toFloat();

    if (checksum(Lspeed, Rspeed, sum)) {
      omega_target_L = Lspeed;
      omega_target_R = Rspeed;
      lastRecvMilli = millis();
    }

  }

  if (end_index != -1 || recbuf.length() > 50) {
    recbuf = "";
  }
}

//!!!
//void sendFeedback_wheel_angularVel() {
//  // getMotorData();
//  // convert rad/s to 16 bit integer to send
//  int actual_send_R = int(omega_actual_R / (double(MaxSpeed) / 32767));
//  int actual_send_L = int(omega_actual_L / (double(MaxSpeed) / 32767));
//  // int kpvalue = pid.readKp();
//  byte buf[6];
//  buf[0] = '{';                   // send start byte
//  buf[1] = highByte(actual_send_R); // send high byte     highByte(actual_send_R)
//  buf[2] = lowByte(actual_send_R);  // send low byte 
//  buf[3] = highByte(actual_send_L);
//  buf[4] = lowByte(actual_send_L);
//  buf[5] = '}'; // send stop byte
//  Serial.write(buf, sizeof(buf));
//}

void sendFeedBack() {
  String speedFb, currentFb, sumFb, gyro;
  float sum;
  if (recieveComplete) {
  speedFb =
      "ls" + String(omega_actual_L) + "rs" + String(omega_actual_R);
  //currentFb = "lc" + String(current_left) + "rc" + String(current_right);
  sum = omega_actual_L + omega_actual_R;
  sumFb = "sum" + String(sum);
  //gyro = "y" + String(yaw.Data);
  feedback = speedFb + sumFb + "end";  //feedback = speedFb + sumFb + gyro + "end";
  Serial.print(feedback);
  recieveComplete = false;
  }
}

void getMotorData() {
  static long EncoderposPreR = 0;
  static long EncoderposPreL = 0;
  // converting ticks/s to rad/s
  omega_actual_R = ((EncoderposR - EncoderposPreR) * (1000 / dT)) * 2 * PI /
                 (CPR * gear_ratio);
  omega_actual_L = ((EncoderposL - EncoderposPreL) * (1000 / dT)) * 2 * PI /
                 (CPR * gear_ratio);
  EncoderposPreR = EncoderposR;
  EncoderposPreL = EncoderposL;
}

void doEncoderR() {
  //Serial.println("doEncoderR");
    pinAState = digitalRead(encoder1PinA);
    pinBState = digitalRead(encoder1PinB);
  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      EncoderposR++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      EncoderposR--;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      EncoderposR++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      EncoderposR--;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      EncoderposR++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      EncoderposR--;
  }
  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      EncoderposR++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      EncoderposR--;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
  //Serial.println(EncoderposR);
}

void doEncoderL() {
  //Serial.println("doEncoderL");
    pinAState = digitalRead(encoder2PinA);
    pinBState = digitalRead(encoder2PinB);
  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      EncoderposL++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      EncoderposL--;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      EncoderposL++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      EncoderposL--;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      EncoderposL++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      EncoderposL--;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      EncoderposL++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      EncoderposL--;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
  //Serial.println(EncoderposL);
}

void give_PWM(double omega_target,int PWM_val,int In_1,int In_2) {
  if (omega_target == 0) {
      PWM_val = 0;
    }
    if (PWM_val == 0) {
      analogWrite(In_1, 0);
      analogWrite(In_2, 0);
    }
    if (PWM_val > 0) {
      analogWrite(In_1, abs(PWM_val));
      analogWrite(In_2, 0);
    }
    if (PWM_val < 0) {
      analogWrite(In_2, abs(PWM_val));
      analogWrite(In_1, 0);
    }
}

void setup() {
  // TCCR0B = TCCR0B & B11111000 | B00000010;
  // A,B:right motor / C,D:left motor
  pinMode(encoder1PinA, INPUT);  //write High when not connected
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);  
  pinMode(encoder2PinB, INPUT);
  
  // encoder pin on interrupt pin 2,3,20,21
  // 20,21: right / 2,3: left
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoderL, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoderL, CHANGE);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  Serial.begin(57600);      //!!!
  // Serial.begin(57600);
  digitalWrite(EnA, HIGH);
  digitalWrite(EnB, HIGH);
  double Kp = 30;
  double Ki = 0.3;
  double Kd = 0;

  pid.setPID(Kp, Ki, Kd);
  pid.setboundary(MaxPWM, -(MaxPWM));
  pid.setMaxSpeedPWM(MaxSpeed);


}

void loop() {
    //readCmd_wheel_angularVel();    //!!!
    
    readCMD();                   // read from odroid
    
  if ((millis() - lastMilli) >= LOOPTIME) { // enter timed loop
    dT = millis() - lastMilli;
    lastMilli = millis();

    getMotorData();    // calculate speed
    //sendFeedback_wheel_angularVel(); // send actually speed to mega   //!!!
    sendFeedBack();   // send to odroid
    
    
    // compute PWM value from rad/s
    PWM_val_R = int(pid.calPID(omega_target_R, omega_actual_R, dT));
    PWM_val_L = int(pid.calPID(omega_target_L, omega_actual_L, dT));

    give_PWM(omega_target_R,PWM_val_R,In2,In1); // right motor rotate CW
    give_PWM(omega_target_L,PWM_val_L,In3,In4); // left motor rotate CCW

  }
}
