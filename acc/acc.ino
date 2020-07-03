#include <Wire.h>

unsigned long tCount=0;
unsigned long tCountPre=0;

float MAX = 0;
const float ts = 0.001*0.001;
const int MPU=0x68;  //MPU 6050 의 I2C 기본 주소

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float dt;
float y=0;
float int_y=0;
float pre_y=0;
float pre_value1=0;
float T=0;  

float standard = 0;
int cp = 100;
int cnt_standard = 1000;
int cnt = 0;

float alpha = 0.8;
float pre_AcZ = 0;
float AcZ_lpf=0;  

void setup(){
  pinMode(8, OUTPUT);
  Wire.begin();      //Wire 라이브러리 초기화I
  Wire.beginTransmission(MPU); //MPU로 데이터 전송 시작
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     //MPU-6050 시작 모드로
  Wire.endTransmission(true); 
  Serial.begin(9600);
}

void saveStatesForNextStep()
{
  tCountPre = tCount;
  pre_y = y;
}

void filter()
{
  float value1 = pre_y*dt + (y-pre_y)*dt/2;
  float value2 = pre_value1*dt + (value1-pre_value1)*dt/2;
  pre_value1 = value1;
  
  if(cnt == 0)
    cnt++;
  
  else
  {
    int_y += value2;
    if( abs(value2) >= 0.02)
      T += dt;
    else
      T=0;
  }
  
  if(abs(MAX) < abs(int_y))
    MAX = int_y*61/1000;
}

void intCalculate()
{
  tCount = micros();
  dt=(float)(tCount - tCountPre)*ts;
  y = AcZ_lpf - standard;
}

void LPF_Filter()
{
  AcZ_lpf = AcZ;
  AcZ_lpf = alpha * pre_AcZ + (1-alpha) * AcZ_lpf;
  pre_AcZ = AcZ_lpf;
/*
  Serial.print(AcZ);
  Serial.print(',');
  Serial.println(AcZ_lpf);
  */
}

void loop()
{
  Wire.beginTransmission(MPU);    //데이터 전송시작
  Wire.write(0x3B);               // register 0x3B (ACCEL_XOUT_H), 큐에 데이터 기록
  Wire.endTransmission(false);    //연결유지
  Wire.requestFrom(MPU,14,true);  //MPU에 데이터 요청
  //데이터 한 바이트 씩 읽어서 반환
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  LPF_Filter();

  if( cp > 0 )
    cp--;
 
  else
  {
    if(cnt_standard>0)
    {  
      standard += AcZ_lpf;
      cnt_standard--;
    }
      
    else if (cnt_standard==0)
    {
      standard /= 1000.0;
      cnt_standard--;
    }
    
    else
    {
      digitalWrite(8, HIGH);
      intCalculate();
      filter();
      
      Serial.println(MAX);
      
      saveStatesForNextStep();
    }
  }
}
