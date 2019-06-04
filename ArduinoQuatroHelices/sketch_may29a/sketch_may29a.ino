#include<Wire.h> // This library allows you to communicate with I2C / TWI devices.


const int MPU = 0x68; //Adress I2C do MPU6050

//Global variables
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int Pitch, Roll, Yaw;
float elapsedTime, time, timePrev;

int sampleTime = 0.5;


# define pinoPWMLF 5  //pino do Arduino que terá a ligação para o driver de motor
# define pinoPWMLB 10
# define pinoPWMRF 6
# define pinoPWMRB 9

int SumOfErrorsPitch = 1, SumOfErrorsRoll = 1 ;
int lastPitch = 1, lastRoll = 1;
double iTermPitch, iTermRoll;

int setPointPitch = 0 , setPointRoll = 0;
float previousErrorPitch = 0, previousErrorRoll = 0;
float pwmLeftFront,pwmLeftBack, pwmRightFront, pwmRightBack;

int ct=0,dt=0,pt=0;

float kP_Pitch = 0; //0.6
float kI_Pitch = 0; //0.00070
float kD_Pitch = 0; //0.00

float kP_Roll = 1; //0.6
float kI_Roll = 0.32; //0.00070
float kD_Roll = 0.43; //0.00

double throttle = 150;



void setup()
{
  Serial.begin(9600);

  time = millis();
  
  pinMode(pinoPWMLF, OUTPUT); //configura como saída pino terá a ligação para o driver de motor
  pinMode(pinoPWMLB, OUTPUT);
  pinMode(pinoPWMRF, OUTPUT); 
  pinMode(pinoPWMRB, OUTPUT);

  unsigned long now = millis();

  //initializes MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);

  //initializes o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);

}

void loop()
{

  
  FunctionsMPU(); // Acquisisco assi AcX, AcY, AcZ.
  ct=millis();
  dt=ct-pt;

  if(dt>=sampleTime)
  {
     pt=ct;
  PIDControl();
  
  }
  Roll = FunctionsPitchRoll(AcX, AcY, AcZ); //Calcolo angolo Roll
  Pitch = FunctionsPitchRoll(AcY, AcX, AcZ); //Calcolo angolo Pitch
  Yaw = FunctionsPitchRoll(AcZ, AcX, AcY); //Calcolo angolo Pitch


  Serial.print("Pitch: "); Serial.print(Pitch);
  Serial.print(" |");
  Serial.print(" Roll: "); Serial.print(Roll);
  Serial.print(" |");
  Serial.print(" Yaw: "); Serial.print(Yaw);
  Serial.println(" |");


}

// Função para o cálculo dos ângulos de inclinação e rotação
double FunctionsPitchRoll(double A, double B, double C) {
  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B * B) + (C * C);
  DatoB = sqrt(DatoB);

  Value = atan2(DatoA, DatoB);
  Value = Value * 180 / 3.14;

  return (int)Value;
}

// Função para aquisição dos eixos X, Y, Z do MPU6050
void FunctionsMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
}

void PIDControl() {
    
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;    

if(elapsedTime>=sampleTime){

  
    float errorPitch = (setPointPitch - Pitch);
    float errorRoll = (setPointRoll - Roll);
    
    float dPitch = (Pitch - lastPitch);
    float dRoll = (Roll - lastRoll);
    
    iTermPitch += (kI_Pitch * errorPitch);
    iTermRoll += (kI_Roll * errorRoll);
    
    if(iTermPitch> 2) iTermPitch= 2;
      else if(iTermPitch<0.1) iTermPitch= 0.1;
    
    if(iTermRoll> 2) iTermRoll= 2;
      else if(iTermRoll<0.1) iTermRoll= 0.1;

      
    /*PID PITCH*/
    float PID_Pitch = kP_Pitch * errorPitch + iTermPitch + kD_Pitch * dPitch;

  if (PID_Pitch < -255){ PID_Pitch = -255; }
  if (PID_Pitch > 255) { PID_Pitch = 255;}



    /*PID ROLL*/
    float PID_Roll = kP_Roll * errorRoll + iTermRoll + kD_Roll * dRoll;

  if (PID_Roll < -255){ PID_Roll = -255; }
  if (PID_Roll > 255) { PID_Roll = 255;}

  



  
  pwmLeftFront = throttle + PID_Pitch - PID_Roll ;
  pwmLeftBack = throttle - PID_Pitch -PID_Roll ;
  pwmRightFront = throttle + PID_Pitch + PID_Roll ;
  pwmRightBack = throttle - PID_Pitch + PID_Roll  ;
  




  
  //Right
  if (pwmRightFront < 51) { pwmRightFront = 51;}
  if (pwmRightFront > 255){ pwmRightFront = 255;}
  if (pwmRightBack < 51) { pwmRightBack = 51;}
  if (pwmRightBack > 255){ pwmRightBack = 255;}
  //Left
  if (pwmLeftFront < 51) { pwmLeftFront = 51;}
  if (pwmLeftFront > 255){ pwmLeftFront = 255;}
  if (pwmLeftBack < 51) { pwmLeftBack = 51;}
  if (pwmLeftBack > 255){ pwmLeftBack = 255;}
  
  
  
  
  
  
  
  Serial.print("Left Front:"); Serial.print(pwmLeftFront);
  Serial.print(" Right Front:"); Serial.println(pwmRightFront);
  Serial.print("Left Back:"); Serial.print(pwmLeftBack);
  Serial.print(" Right Back:"); Serial.println(pwmRightBack);

  analogWrite(pinoPWMLF, 150 /*pwmLeftFront */);
  analogWrite(pinoPWMLB, 150 /*pwmLeftBack*/);
  analogWrite(pinoPWMRF, 150 /*pwmRightFront*/);
  analogWrite(pinoPWMRB, 150 /*pwmRightBack*/);




  
    /*Pitch*/
    lastPitch = Pitch;
    previousErrorPitch = errorPitch;
    SumOfErrorsRoll += errorPitch;    
    /*Roll*/
    lastRoll = Roll;
    previousErrorRoll = errorRoll;
    SumOfErrorsRoll += errorRoll;      


    timePrev = time;  // the previous time is stored before the actual time read
   }
}
 
