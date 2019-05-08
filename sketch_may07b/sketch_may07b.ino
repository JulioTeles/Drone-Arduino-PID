//Carrega a biblioteca Wire
#include<Wire.h> // This library allows you to communicate with I2C / TWI devices.
#include<Servo.h>

const int MPU = 0x68; //Endereco I2C do MPU6050

//Variaveis globais
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int Pitch, Roll, Yaw; // inclinação e rotação
float elapsedTime, time, timePrev;

Servo right_prop;
Servo left_prop;

double throttle=1300;

void setup()
{
  Serial.begin(9600);
  
  time = millis();
   
  //Inicializa o MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);

  //Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop()
{
  FunctionsMPU(); // Acquisisco assi AcX, AcY, AcZ.
  
  PIDControl();
  
  Roll = FunctionsPitchRoll(AcX, AcY, AcZ); //Calcolo angolo Roll
  Pitch = FunctionsPitchRoll(AcY, AcX, AcZ); //Calcolo angolo Pitch
  Yaw = FunctionsPitchRoll(AcZ, AcX, AcY); //Calcolo angolo Pitch


  Serial.print("Pitch: "); Serial.print(Pitch);
  Serial.print("|");
  Serial.print("Roll: "); Serial.print(Roll);
  Serial.print("|");
  Serial.print("Yaw: "); Serial.print(Yaw);
  Serial.print("|");  
  Serial.print("\n");

  

  delay(3000);

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
  AcX = Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
}

int sommeErreurPitch = 1;
int sommeErreurRoll = 1;
int setPointPitch = 0;
float previousErrorPitch = 0;
float pwmLeft, pwmRight;

void PIDControl(){
  float kP = 1;
  float kI = 1;
  float kD = 1;
  float errorPitch = (setPointPitch-Pitch);

  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 
   

  float PID = kP*errorPitch + kI*sommeErreurPitch + kD*((errorPitch-previousErrorPitch)/elapsedTime);

  
  if(PID < -1000)
    {
      PID=-1000;
    }
  if(PID > 1000)
    {
      PID=1000;
     }

     pwmLeft = throttle + PID;
      pwmRight = throttle - PID;

    if(pwmRight < 1000)
    {
      pwmRight= 1000;
    }
    if(pwmRight > 2000)
    {
      pwmRight=2000;
    }
    //Left
    if(pwmLeft < 1000)
    {
      pwmLeft= 1000;
    }
    if(pwmLeft > 2000)
    {
      pwmLeft=2000;
    }

     Serial.print("Left:"); Serial.println(pwmLeft);
     Serial.print("Right:");Serial.println(pwmRight);
  previousErrorPitch = errorPitch;
  sommeErreurPitch += errorPitch;
  
}
