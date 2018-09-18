#include <Wire.h>
#include <MyoBridge.h>
#include <SoftwareSerial.h>
SoftwareSerial bridgeSerial(2,3);
MyoBridge bridge(bridgeSerial);


//for time
unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;


//for average;
const int numReadings = 5;
float readings[8][numReadings];
int readIndex = 0;
float total[8] = {0,0,0,0,0,0,0,0};
float emgcal[8] = {0,0,0,0,0,0,0,0};


//pre-declare
void predict(struct EMGKalman *kalman, float vEmg, float dt);
float update(struct EMGKalman *kalman, float emg_m);

//kalman filter structure
struct EMGKalman{
  float x_emgval, emg_bias;
  float P_00, P_01, P_10, P_11;

  
  float Q_emg, Q_val;



  float R_emg;
};


//defines as matrix
struct EMGKalman emg[8];


int emg_1_[8] = {0,0,0,0,0,0,0,0};
float emg_2_[8] = {0,0,0,0,0,0,0,0};


static const float def_R_emg = 0.3; //0.3 default
static const float def_Q_emg = 0.01; //.01 default
static const float def_Q_val = 0.04; //.04 default


int emginit[8][5];
int initIndex = 0;
int initSize = 5;
int xCal = 0;
int yCal = 0;
int zCal = 1800;




///////////////////////////////////////
///// EMG data OUT function
////////////////////////////////////////
void handleEMGData(int8_t data[8]) {
  curSensoredTime = millis();

// 칼만필터 적용 루틴
  if(prevSensoredTime > 0){
    
  int emg_1_[8];
  float emg_2_[8];
  for(int i=0;i<8;i++)
  {
    emg_2_[i]=0;
    emg_1_[i]=0;
  }

  
  int loopTime = curSensoredTime - prevSensoredTime;


  
  for(int i=0;i<8;i++){

  emg_2_[i]= float(data[i]);
  predict(&emg[i], emg_2_[i], loopTime);
  
  emg_1_[i] = kupdate(&emg[i], data[i]) / 10;
  }




// 값 평균내는 루틴
if(readIndex < numReadings){
    for(int i=0;i<8;i++){
      readings[i][readIndex]=emg_1_[i];
      if(readIndex == numReadings -1){
        total[i] = 0;
        for(int j=0;j<numReadings;j++){
           total[i] += readings[i][j];
        }

        emgcal[i] -= total[i]/(numReadings-1);
      }
    }
    readIndex++;
  }
  else{
  for(int i=0;i<8;i++)
  emg_1_[i]+=emgcal[i];
  }


  for(int i=0;i<8;i++){
  Serial.print("EMGval[");
  Serial.print(i);
  Serial.print("] is ");
  Serial.print(emg_1_[i]);
  }
  Serial.println("");



  prevSensoredTime = curSensoredTime;
}
}
////////////////
////// 설정하기 (초기화 등)
/////////////////
void setup() {
  Serial.begin(115200);
  bridgeSerial.begin(115200);
  for(int i=0;i<8;i++){
    initEMGKalman(&emg[i],def_Q_emg,def_Q_val,def_R_emg);
    
  
   for(int j=0;j<numReadings;j++)
  {
    readings[i][j]=0;
  }
  }
  Serial.println("Searching for Myo...");
  bridge.begin();
  Serial.println("connected!");

  bridge.setEMGDataCallBack(handleEMGData);
  bridge.setEMGMode(EMG_MODE_SEND);
  bridge.disableSleep();
}

void loop() {
  bridge.update();
}








/////////////////
//// 칼만 초기화
////////////////////
void initEMGKalman(struct EMGKalman *kalman, const float Q_emg, const float Q_val,  const float R_emg) {

  kalman->Q_emg = Q_emg;
  kalman->Q_val = Q_val;
  kalman->R_emg = R_emg;
  
  kalman->P_00 = 0;
  kalman->P_01 = 0;
  kalman->P_10 = 0;
  kalman->P_11 = 0;
}
/*
* The kalman predict method.
* kalman    the kalman data structure
* dotAngle    Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
*           This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
*/
void predict(struct EMGKalman *kalman, float vEmg, float dt) {
  kalman->x_emgval += dt * (vEmg - kalman->emg_bias);
  kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_emg ;
  kalman->P_01 += -1 * dt * kalman->P_11;
  kalman->P_10 += -1 * dt * kalman->P_11;
  kalman->P_11 += kalman->Q_val;
}

/*
* The kalman update method
* kalman  the kalman data structure
* angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
*/
float kupdate(struct EMGKalman *kalman, float emg_m) {
  const float y = emg_m - kalman->x_emgval;
  const float S = kalman->P_00 + kalman->R_emg;
  const float K_0 = kalman->P_00 / S;
  const float K_1 = kalman->P_10 / S;
  kalman->x_emgval += K_0 * y;
  kalman->emg_bias += K_1 * y;
  kalman->P_00 -= K_0 * kalman->P_00;
  kalman->P_01 -= K_0 * kalman->P_01;
  kalman->P_10 -= K_1 * kalman->P_00;
  kalman->P_11 -= K_1 * kalman->P_01;
  return kalman->x_emgval;
}


