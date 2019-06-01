#include<Wire.h> // This library allows you to communicate with I2C / TWI devices.


const int MPU = 0x68; //Adress I2C do MPU6050

//Global variables
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int Pitch, Roll, Yaw;
float elapsedTime, time, timePrev;

# define pinoPWML 5  //pino do Arduino que terá a ligação para o driver de motor
# define pinoPWMR 3

int sommeErreurPitch = 1;
int setPointPitch = 0;
float previousErrorPitch = 0;
float pwmLeft, pwmRight;

float kP = 1.3;
float kI = 0.0;
float kD = 0.0;

double throttle = 150;

void setup()
{
  Serial.begin(9600);

  pinMode(pinoPWML, OUTPUT); //configura como saída pino terá a ligação para o driver de motor
  pinMode(pinoPWMR, OUTPUT);

  time = millis();

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

  float errorPitch = (setPointPitch - Pitch);

  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;


  float PID = kP * errorPitch + kI * sommeErreurPitch + kD * ((errorPitch - previousErrorPitch) / elapsedTime);
  
  if (PID < -255)
    {
      PID = -255;
    }
  if (PID > 255)
    {
      PID = 255;
    }

  pwmLeft = throttle + PID;
  pwmRight = throttle - PID;

  if (pwmRight < 51)
    {
      pwmRight = 51;
    }
  if (pwmRight > 255)
    {
      pwmRight = 255;
    }
  //Left
  if (pwmLeft < 51)
    {
      pwmLeft = 51;
    }
  if (pwmLeft > 255)
    {
      pwmLeft = 255;
    }

  Serial.print("Left:"); Serial.print(pwmLeft);
  Serial.print("Right:"); Serial.println(pwmRight);

  analogWrite(pinoPWML, pwmLeft*0.6);
  analogWrite(pinoPWMR, pwmRight);

  
  previousErrorPitch = errorPitch;
  sommeErreurPitch += errorPitch;

}
