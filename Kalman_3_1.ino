#include <MyoBridge.h>
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>
//SoftwareSerial connection to MyoBridge
SoftwareSerial bridgeSerial(2,3);//connected to BT


float e_mea = 8;
float e_est = 1;
float noise = 2.4;
float ppow = 1.7;


//////////////////////////////
const int numReadings = 5;
float readings[8][numReadings];//8x5 array for what?
float total[8] = {0,0,0,0,0,0,0,0};
float average[8];
int readIndex=0;
///////////////////////////////


///sdsdsdsdsdsd//////////////////
SimpleKalmanFilter kalman_0(e_mea,e_est,noise,ppow);//KalmanFiltering with some values
SimpleKalmanFilter kalman_1(e_mea,e_est,noise,ppow);
SimpleKalmanFilter kalman_2(e_mea,e_est,noise,ppow);
SimpleKalmanFilter kalman_3(e_mea,e_est,noise,ppow);
SimpleKalmanFilter kalman_4(e_mea,e_est,noise,ppow);
SimpleKalmanFilter kalman_5(e_mea,e_est,noise,ppow);
SimpleKalmanFilter kalman_6(e_mea,e_est,noise,ppow);
SimpleKalmanFilter kalman_7(e_mea,e_est,noise,ppow);
///////////////sdsdsdsdsds////////



//initialize MyoBridge object with software serial connection
MyoBridge bridge(bridgeSerial);//(2,3) bridge
  float filtered[8] = {0,0,0,0,0,0,0,0};
  float emgval[8]={0,0,0,0,0,0,0,0};
//declare a function to handle EMG data

void handleEMGData(int8_t data[8]) { //여기서 값을 받아서 필터링 하는거 같은데 
  
   emgval[0]=abs(data[0]);
   filtered[0]=kalman_0.updateEstimate(emgval[0]);
   emgval[1]=abs(data[1]);
   filtered[1]=kalman_1.updateEstimate(emgval[1]);
   emgval[2]=abs(data[2]);
   filtered[2]=kalman_2.updateEstimate(emgval[2]);
   emgval[3]=abs(data[3]);
   filtered[3]=kalman_3.updateEstimate(emgval[3]);
   emgval[4]=abs(data[4]);
   filtered[4]=kalman_4.updateEstimate(emgval[4]);
   emgval[5]=abs(data[5]);
   filtered[5]=kalman_5.updateEstimate(emgval[5]);
   emgval[6]=abs(data[6]);
   filtered[6]=kalman_6.updateEstimate(emgval[6]);
    emgval[7]=abs(data[7]);
   filtered[7]=kalman_7.updateEstimate(emgval[7]);
   
   /*
emgval[3]=abs(data[3]);
   filtered[1]=kalman_8.updateEstimate(emgval[3]);

emgval[4]=abs(data[3]);
   filtered[2]=kalman_9.updateEstimate(emgval[4]);

 
  Serial.print(filtered[0]);
  Serial.print(" , ");

  
  Serial.print(filtered[1]);
  Serial.print(" , ");
  Serial.print(filtered[2]);
  Serial.print(" , ");
   Serial.print(filtered[3]);
  Serial.print(" , ");
  Serial.print(filtered[4]);
  Serial.print(" , ");
  Serial.print(filtered[5]);
  Serial.print(" , ");
  Serial.print(filtered[6]);
  Serial.print(" , ");
  Serial.print(filtered[7]);
   Serial.println("");
  //the EMG data is 8bit signed integers. Just print them out:

  
  /*
  Serial.print(filtered[3]);
  Serial.print(" , ");
  Serial.print(emgval[3]);
  Serial.print(" , ");
  Serial.print(data[3]);
*/
  
}

void setup() {
  //initialize both serial connections
  Serial.begin(115200);
  bridgeSerial.begin(115200);

  //wait until MyoBridge has found Myo and is connected. Make sure Myo is not connected to anything else and not in standby!
  Serial.println("Searching for Myo...");
  bridge.begin();
  Serial.println("connected!");

  //set the function that handles EMG data
  bridge.setEMGDataCallBack(handleEMGData);
  //tell the Myo we want the filtered EMG data
  bridge.setEMGMode(EMG_MODE_SEND);
  //disable sleep mode, so we get continous data even when not synced 
  bridge.disableSleep();
}

void loop() {
  //update the connection to MyoBridge
  bridge.update();//미요에서 값을 계속 받는 건가. 근데 받기만 하고 슬레이브에 보내는건 하나도 없는데
}
