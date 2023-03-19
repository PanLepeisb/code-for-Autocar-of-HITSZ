#include <MsTimer2.h>
#include <Servo.h>

#define ENCODER_A1 2
#define ENCODER_B1 4
#define ENCODER_A2 3
#define ENCODER_B2 5

#define PWM1 9
#define PWM2 10

#define L1 26
#define L2 24
#define L3 22
#define R1 49
#define R2 51
#define R3 53
#define M  40

#define PERIOD 20//8

#define Kp1 10.0
#define Ti1 40.0 
#define Td1 6.0
#define Kp2 18.0
#define Ti2 20.0
#define Td2 8.0

#define SERVO_A_BASE 160
#define SERVO_B_BASE 0
#define SERVO_A_DOWN 100
#define SERVO_B_PICK 50
#define SERVO_A_PIN 12 //俯仰
#define SERVO_B_PIN 11 //爪子
Servo servoA, servoB;

float target1,t1;
float target2,t2;
volatile long encoderVal1;
float velocity1;
volatile long encoderVal2;
float velocity2;
float T=PERIOD;

float q01=Kp1*(1+T/Ti1+Td1/T);
float q11=-Kp1*(1+2*Td1/T);
float q21=Kp1*Td1/T;
float q02=Kp2*(1+T/Ti2+Td2/T);
float q12=-Kp2*(1+2*Td2/T);
float q22=Kp2*Td2/T;

float u1,ek11,ek12;
float u2,ek21,ek22;

float V=-12.0;//16.0

// int flagl=0;
// int flagr=0;
int temp1=0;
int temp2=0;

void control(void)
{

  if (digitalRead(M)==HIGH)
  {
    target1=V;
    target2=V;
  }
  if (digitalRead(L1)==HIGH)
  {
    target1=V*1.1;
    target2=V*0.4;
  }
  if (digitalRead(R1)==HIGH)
  {
    target1=V*0.3;
    target2=V*1.5;
  }
  if (digitalRead(L2)==HIGH)
  {
    target1=V*1.1;
    target2=V*0.2;
  }
  if (digitalRead(R2)==HIGH)
  {
    target1=V*0;
    target2=V*1.75;
  }
  if (digitalRead(L3)==HIGH)
  {
    target1=V*1.1;
    target2=0;
  }
  if (digitalRead(R3)==HIGH)
  {
    target1=0;
    target2=V*2.7;   
  }
  if ((digitalRead(L1)==HIGH)&&(digitalRead(L2)==HIGH)&&(digitalRead(L3)==HIGH))
  {
    // target1=V*2.0;
    // target2=0;
    // flagl=1;
    temp1=1;
  }
  if ((digitalRead(R1)==HIGH)&&(digitalRead(R2)==HIGH)&&(digitalRead(R3)==HIGH))
  {
    // target1=V*0;
    // target2=V*5.0;
    // flagr=1;
    temp2=1;
  }
  
  target1=target1*0.8+t1*0.2;
  target2=target2*0.85+t2*0.15;
  velocity1=(encoderVal1/780.0)*3.1415*2.0*(1000/PERIOD);
  encoderVal1=0;
  velocity2=(encoderVal2/780.0)*3.1415*2.0*(1000/PERIOD);
  encoderVal2=0;
  int output1=pidController1(target1,velocity1);
  int output2=pidController2(target2,velocity2);
  analogWrite(PWM1,(output1+255)/2);
  analogWrite(PWM2,(output2+255)/2);
  if (temp1==1)
  {
    analogWrite(PWM1,255);
    analogWrite(PWM2,0);
    delay(900);
    temp1=0;      
  }
  if (temp2==1)
  {
    analogWrite(PWM1,0);
    analogWrite(PWM2,255);
    delay(800);
    temp2=0;
  }
    if ((digitalRead(L3)==HIGH)&&(digitalRead(R3)==HIGH))
  {      
    analogWrite(PWM1,255);
    analogWrite(PWM2,0);
    delay(50000); 
    delay(10000); 
    V=1;
    target1=0;
    target2=0;
    analogWrite(PWM1,255);
    analogWrite(PWM2,255);
    delay(20000);
    Arm_drop();
  }  
  // if (flagl==1)
  // {
  //   delay(500);
  //   flagl=0;    
  // }
  // if (flagr==1)
  // {
  //   delay(250);
  //   flagr=0;
  // }
  t1=target1;
  t2=target2;
}
void setup() 
{  
  // pinMode(ENA,OUTPUT);
  // digitalWrite(ENA,LOW);
  TCCR1B=TCCR1B&B11111000|B00000001;
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  analogWrite(PWM1,128);
  analogWrite(PWM2,128);
  delay(1000);
  pinMode(ENCODER_A1,INPUT);
  pinMode(ENCODER_B1,INPUT);
  pinMode(ENCODER_A2,INPUT);
  pinMode(ENCODER_B2,INPUT);
  attachInterrupt(0,getEncoder1,CHANGE);
  attachInterrupt(1,getEncoder2,CHANGE);
  Serial.begin(9600);
  pinMode(R1,INPUT);
  pinMode(R2,INPUT);
  pinMode(R3,INPUT);
  pinMode(L1,INPUT);
  pinMode(L2,INPUT);
  pinMode(L3,INPUT);
  pinMode(M,INPUT);
  Arm_setup();
  Arm_pickup();
  delay(1000); 
  MsTimer2::set(PERIOD,control);
  MsTimer2::start();  
}

void loop()
{
}
void getEncoder1(void)
{
  if (digitalRead(ENCODER_A1)==LOW)
  {
    if (digitalRead(ENCODER_B1)==LOW)
    {
      encoderVal1++;      
    }
    else
    {
      encoderVal1--;
    }
  }
  else
  {
    if (digitalRead(ENCODER_B1)==LOW)
    {
      encoderVal1--;
    }
    else
    {
      encoderVal1++;
    }
  }
}
void getEncoder2(void)
{
  if (digitalRead(ENCODER_A2)==LOW)
  {
    if (digitalRead(ENCODER_B2)==LOW)
    {
      encoderVal2--;
    } 
    else
    {
      encoderVal2++;
    }   
  }
  else
  {
    if (digitalRead(ENCODER_B2)==LOW)
    {
      encoderVal2++;
    }
    else
    {
      encoderVal2--;
    }
  }
}
int pidController1(float targetVelocity,float currentVelocity)
{
  float ek10;
  ek10=targetVelocity-currentVelocity;
  u1=u1+q01*ek10+q11*ek11+q21*ek12;
  if (u1>255)
  {
    u1=255;
  }
  if (u1<-255)
  {
    u1=-255;
  }
  ek12=ek11;
  ek11=ek10;
  return (int)u1;
}
int pidController2(float targetVelocity,float currentVelocity)
{
  float ek20;
  ek20=targetVelocity-currentVelocity;
  u2=u2+q02*ek20+q12*ek21+q22*ek22;
  if (u2>255)
  {
    u2=255;
  }
  if (u2<-255)
  {
    u2=-255;
  }
  ek22=ek21;
  ek21=ek20;
  return (int)u2;
}
void Arm_setup()
{ 
  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);
  servoA.write(SERVO_A_BASE);
  servoB.write(SERVO_B_BASE);
}
void Arm_pickup()
{
  servoA.write(SERVO_A_DOWN);
  delay(750);
  servoB.write(SERVO_B_PICK);
  delay(500);
  servoA.write(180);
}
void Arm_drop()
{
  servoA.write(SERVO_A_DOWN);
  delay(50000);
  servoB.write(SERVO_B_BASE);
  delay(5000);
}