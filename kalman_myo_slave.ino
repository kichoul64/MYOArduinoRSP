#include <Wire.h>
#include <Servo.h> 
#define SLAVE 4
Servo finger1;      
Servo finger2;
Servo finger3;
Servo finger4;
Servo finger5;
byte data[8];


/*
const int numReadings = 3;
float readings[8][numReadings];
float average[8];
*/

float aver=0;
int readIndex=0;
float total[8]={0,0,0,0,0,0,0,0};
void setup() 
{//setup process 1)fingers.attach(), 2)wire connection and receive
    finger1.attach(3);                
    finger2.attach(5); //역행                
    finger3.attach(6); 
    finger4.attach(9); //역행
    finger5.attach(11);
 Serial.begin(115200);
 Serial.println("begin transmission");

 /*
for(int i=0;i<8;i++){
   for(int j=0;j<numReadings;j++)
 {
    readings[i][j]=0;
 }
}
*/

 Wire.begin(SLAVE);//Wire
 Wire.onReceive(receiveEvent);
}

void receiveEvent()
{ 
  int i=0;
  while(Wire.available())
  {
    data[i]=Wire.read();//byte 단위로 받고
    i++;
  }
  
  Serial.print(data[0]);
  Serial.print(" , ");
  Serial.print(data[1]);
  Serial.print(" , ");
  Serial.print(data[2]);
  Serial.print(" , ");
  Serial.print(data[3]);
  Serial.print(" , ");
  Serial.print(data[4]);
  Serial.print(" , ");
  Serial.print(data[5]);
  Serial.print(" , ");
  Serial.print(data[6]);
  Serial.print(" , ");
  Serial.print(data[7]);
  Serial.println("");
 
   for(int i=0;i<8;i++)
    {
    /*
    total[i] = total[i] - readings[i][readIndex];
    readings[i][readIndex] = data[i];
    total[i] = total[i] + readings[i][readIndex];
    readIndex = readIndex + 1; 
    if(readIndex >= numReadings) {
    readIndex = 0; 
    }
    average[i] = (total[i]/numReadings);
    Serial.print(" ");//이 과정이 Master로 간것인가
    */
 
   aver=aver+data[i];
    }
   aver=aver/10;
   if(aver>30) aver=30;
   int aver2=aver;
  
   
   aver=(int)map(aver,0,30,0,7);
   aver=map(aver,0,7,0,180);  
   int invaver=180-aver;
   Serial.print(aver2);
   
   if(aver2<5.0)
   { //힘이 없으면 보자기
   finger1.write(0);
   finger2.write(180);
   finger3.write(0);
   finger4.write(180);
   finger5.write(0);
   Serial.println("Paper");
   }

   else if(5.0<=aver2 && aver2<20.0 )
   {
   finger1.write(180); 
   finger2.write(180);   
   finger3.write(0);
   finger4.write(0);  
   finger5.write(180);
   Serial.println("Scissor ");
   }
 
   else 
   {
   finger1.write(aver); 
   finger2.write(invaver);   
   finger3.write(aver);
   finger4.write(invaver);  
   finger5.write(aver);
   Serial.println("ROCK ");
   }

   
   
}
void loop() {
}

