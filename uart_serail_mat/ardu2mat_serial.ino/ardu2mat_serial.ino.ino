#include "string.h"
void serialEvent1();
void serialEvent();

float data[8];
float jnt_ang[9]; // float형태

bool st_matlab = false;
bool st_matlab2 = false;
bool st_matlab3 = false;
bool sent = false;
bool robot_op = false;
bool packet = false;
char Sendmessage = 'A';
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.setTimeout(0.001);
}

void loop() {

}
void serialEvent(){
  if (Serial.available() > 0) {
    String temp = Serial.readStringUntil('\n');
    int ball_screw = temp.toInt();
    // if(!robot_op){
      if(ball_screw == 1){ // 3입력 후 실행
        robot_op = true;
        Serial1.println("/25/");
        Serial.println("Operating");
        }
      
      else if(ball_screw == 2){ // 원상 복귀
        Serial1.println("/3/");
        Serial.println("return");
      }
      else if(ball_screw == 3){
        Serial1.println("/10/");
        Serial.println("grip");
      }
      else if(ball_screw == 4){
        Serial1.println("/s/");
        Serial.println("keep_going");
      }
    }
}

void serialEvent1(){
  static char serialInBuffer[32];
  static int data_cnt = 0, buff_cnt = 0; 
  
  if(Serial1.available() > 0) {
    char byteIn = Serial1.read();
    if(byteIn=='['&& !st_matlab){
      st_matlab = true;
      }
    else if(byteIn == 'a' && !st_matlab3 && st_matlab){
      st_matlab2 = true;
    }
    else if(byteIn == 'b' && !st_matlab2 && st_matlab){
      st_matlab3 = true;
    }
    else if(st_matlab2 && byteIn==','){
      // Serial1.println(serialInBuffer);
      serialInBuffer[buff_cnt] = '\0';
      data[data_cnt++] = atof(serialInBuffer);
      buff_cnt = 0;
    }
    else if(st_matlab2 && byteIn == ']'){
      serialInBuffer[buff_cnt] = '\0';
      data[data_cnt] = atof(serialInBuffer);
      buff_cnt = 0; data_cnt = 0;
      st_matlab = false;
      st_matlab2 = false;
      st_matlab3 = false;
      jnt_ang[0] = data[0]; // status
      jnt_ang[1] = data[1]+268.7;
      jnt_ang[2] = data[2]+178.43;
      jnt_ang[3] = data[2]+180.11;
      // jnt_ang[4] = data[3]+183.87;
      // jnt_ang[5] = data[4]+177.82;
      // jnt_ang[6] = data[5]+270.99;
      // jnt_ang[7] = data[6]+197.67;
      jnt_ang[4] = data[3]+181.37;
      jnt_ang[5] = data[4]+178.3;
      jnt_ang[6] = data[5]+270;
      jnt_ang[7] = data[6]+197.67;
      packet = true;
      for(int i = 0; i< 8; i++)Serial.println(jnt_ang[i]);
      }

    else if(st_matlab3 && byteIn==','){
      // Serial1.println(serialInBuffer);
      serialInBuffer[buff_cnt] = '\0';
      data[data_cnt++] = atof(serialInBuffer);
      buff_cnt = 0;
    }
    else if(st_matlab3 && byteIn == ']'){
      serialInBuffer[buff_cnt] = '\0';
      data[data_cnt] = atof(serialInBuffer);
      buff_cnt = 0; data_cnt = 0;
      st_matlab = false;
      st_matlab2 = false;
      st_matlab3 = false;
      Serial.println(data[1]);
      }
    else{
      serialInBuffer[buff_cnt++] = byteIn;
    }
  }
}
