#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"
#define TCAADDR 0x70
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

#define RESTRICT_PITCH 

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

MPU6050 accelGyroMag;

int MPU1 = 0;
int MPU2 = 1;
int MPU3 = 2;
int MPU4 = 3;

const int MPU = 0x68; // The code for the mpu-6050 is from Arduino User JohnChi
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

int16_t AX, AY, AZ;
int16_t GX, GY, GZ;
int16_t MX, MY, MZ;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double roll , pitch;

double gyroXrate , gyroYrate;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

const int totalIMU = 4; 

double ax[totalIMU] ,ay[totalIMU] ,az[totalIMU];
double gx[totalIMU] , gy[totalIMU] , gz[totalIMU];
double mx[totalIMU] , my[totalIMU] , mz[totalIMU];
double ROLL[totalIMU] , PITCH[totalIMU];

double dt;
float deltat = 0.0f;

uint32_t timer;
int inByte = 0;         // incoming serial byte

// the multiplexer address select lines (A/B/C)
const byte addressA = 6; // low-order bit A0
const byte addressB = 5;  // A1
const byte addressC = 4; // high-order bit A2



void setup()
{
  pinMode(addressA, OUTPUT);  //Lower Bit-6
  pinMode(addressB, OUTPUT);  //         -5
  pinMode(addressC, OUTPUT);  //Higher Bit-4
  
  Wire.begin(); // wake up I2C bus
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  restrictedPitch(accX , accY, accZ);
#else // Eq. 28 and 29
  unrestrictedPitch(accX , accY , accZ);
#endif
  setStartingAngle(roll,pitch);
  
}


void loop()
{
    dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
     
  //while(Serial.available() > 0)
  {
  
 
  
       
        // First IMU:
        //selectTarget(MPU1);
         tcaselect(1); read_imus(1);
         tcaselect(7); read_imus(2);
         //Serial.print("Reading first IMU"); Serial.print(" \t "); Serial.print("Reading Second IMU");
         //Serial.print("Roll_1");  Serial.print(" \t "); 
         Serial.print(ROLL[1]);
         //Serial.print("Pitch_1");  
         Serial.print(" \t "); 
         Serial.print(PITCH[1]);
         //Serial.print("Roll_2");  
         Serial.print(" \t "); 
         Serial.print(ROLL[2]);
         //Serial.print("Pitch_2");  
         Serial.print(" \t "); 
         Serial.print(PITCH[2]);
         Serial.println();
      
        //Serial.print("\t");
        //delay(5);
        // Second IMU:
         
      
        
       // selectTarget(MPU2);   
        //Serial.print("\t");
        //delay(5);
        // Third IMU:
        //selectTarget(MPU3);   //read_imus(3);
       // Serial.print("\t");
        //delay(5);
        // Forth IMU:
        //selectTarget(MPU4);   //read_imus(4);
        //Serial.print("\t");
        //delay(5);
       
   
  }     
}

/*int selectTarget (const byte which)
{
  // select correct MUX channel
  digitalWrite (addressA, (which & 1) ? HIGH : LOW);  // low-order bit //Bitwise AND
  digitalWrite (addressB, (which & 2) ? HIGH : LOW);
  digitalWrite (addressC, (which & 4) ? HIGH : LOW);  // high-order bit
} */ 


void read_imus(int a)
{
  int i =a;
   accelGyroMag.initialize();
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  accelGyroMag.getMotion9(&AX, &AY, &AZ, &GX, &GY, &GZ, &MX, &MY, &MZ);
  ax[i] = (double) AX;  ay[i] = (double) AY;  az[i] = (double) AZ;
  gx[i]= (double) GX;  gy[i] = (double) GY;  gz[i] = (double) GZ;
 //Serial.print("ax"); Serial.print(ax[i]); Serial.print("\t");
  //Serial.print("\t");
  //Serial.print("ay"); Serial.print(ay[i]); Serial.print("\t");
  //Serial.print("\t");
   //Serial.print("az"); Serial.print(az[i]); Serial.print("\t");
  //Serial.print("\r\n");
  //delay(2);
 accX = ax[i] ; accY = ay[i] ; accZ = az[i] ;
 gyroX = gx[i]; gyroY = gy[i]; gyroZ = gz[i]; 
 
 
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
restrictedPitch(accX , accY , accZ);
#else // Eq. 28 and 29
 unrestrictedPitch(accX , accY , accZ);
#endif
 convertGyroRate( gyroXrate , gyroYrate);
#ifdef RESTRICT_PITCH
 // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
 restrictedPitchTransition(roll,pitch,gyroXrate,gyroYrate,kalAngleX,kalAngleY,dt);
#else
 // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
 unRestrictedPitchTransition(roll,pitch,gyroXrate,gyroYrate,kalAngleX,kalAngleY,dt);
#endif
  calculateGyroAngle(gyroXrate , gyroYrate ,roll ,pitch ,dt);
 //Printing Roll and Pitch Values
 ROLL[i] = roll; PITCH[i] = pitch;
 /* Serial.print("roll"); Serial.print(roll); Serial.print("\t");
  Serial.print("\t");
  Serial.print("pitch");Serial.print(pitch); Serial.print("\t");
  Serial.print("\r\n");
  delay(2);
 */
  }

  double restrictedPitch(double x ,double y, double z)
  {
    accX = x; accY = y; accZ= z;
   roll  = atan2(accY, accZ) * RAD_TO_DEG;
   pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
   return roll , pitch;
   }

double unrestrictedPitch(double a, double b, double c)
{
  accX = a; accY = b; accZ= c;
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  return roll , pitch;
  
  }

  double setStartingAngle(double r ,double p)
{
  roll = r; pitch=p;
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

 double gyroReset(double x_angle , double y_angle)
  {
    gyroXangle = x_angle ; gyroYangle = y_angle;
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  }

 double restrictedPitchTransition(double r,double p,double gXrate ,double gYrate ,double angleX ,double angleY,double t)
  {
  roll = r; pitch =p; gyroXrate= gXrate; gyroYrate= gYrate; kalAngleX = angleX; kalAngleY = angleY; dt = t;
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) 
  {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } 
    else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
  {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  }
  }

  double  unRestrictedPitchTransition(double r,double p,double gXrate ,double gYrate ,double angleX ,double angleY,double t)
 {
  roll = r; pitch =p; gyroXrate= gXrate; gyroYrate= gYrate; kalAngleX = angleX; kalAngleY = angleY; dt = t;
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
  {
  gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }
 }

double calculateGyroAngle(double xRate , double yRate , double r , double p , double t)
  {
    gyroXrate = xRate; gyroYrate = yRate; roll = r; pitch = p; dt = t;
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
    gyroReset(gyroXangle , gyroYangle);
  }

  double convertGyroRate(double xRate , double yRate)
  {
    gyroXrate = xRate; gyroYrate = yRate;
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  }




