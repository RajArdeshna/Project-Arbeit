//Connect INT pin of IMU6050 to pin 2 of Arduino
//SDA - A4(NANO) , SCL - A5(NANO)
//Multiplexer - NANO
//A0 - 6
//A1 - 5
//A2 - 4

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"
#include "direction_vector.h"
#include "DirectKinematics.h"
#include <MatrixMath.h>
#define TCAADDR 0x70
#define RESTRICT_PITCH
MPU6050 accelGyroMag;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


DirectKinematics DK;

direction_vector v1;
direction_vector v2;
direction_vector v3;



int MPU1 = 0;
int MPU2 = 5;
int MPU3 = 2;
int MPU4 = 3;
int MPU5 = 4;

int Fb_M1 = A1;
int Fb_M2 = A2;
int Fb_M3 = A3;
double Fb1_val=0,Fb2_val=0,Fb3_val=0;
double len1=0,len2=0,len3=0;
double voltage2mm =0.09775171 ;//(100/1023)
int a,i;

const int MPU = 0x68; // The code for the mpu-6050 is from Arduino User JohnChi
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

int16_t AX, AY, AZ;
int16_t GX, GY, GZ;

double accX = 0, accY= 0, accZ= 0;
double gyroX= 0, gyroY= 0, gyroZ= 0;
double roll = 0, pitch= 0;

double gyroXrate , gyroYrate;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

//const int totalIMU = 5;
const int totalIMU = 5; 

double ax[totalIMU] ,ay[totalIMU] ,az[totalIMU];
double gx[totalIMU] , gy[totalIMU] , gz[totalIMU];
double mx[totalIMU] , my[totalIMU] , mz[totalIMU];
double ROLL[totalIMU] , PITCH[totalIMU];
double ROLL_c[totalIMU] , PITCH_c[totalIMU];
double ROLL_raw[totalIMU] , PITCH_raw[totalIMU];
double gyroXrate_comp[totalIMU] ,  gyroYrate_comp[totalIMU];


double vector1[3],vector1_r[3];
double vector2[3],vector2_r[3];
double vector3[3],vector3_r[3];
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
   Serial.begin(115200);
   Wire.begin();
 pinMode(Fb_M1,INPUT);
 pinMode(Fb_M2,INPUT);
 pinMode(Fb_M3,INPUT);  
  pinMode(addressA, OUTPUT);  //Lower Bit-6
  pinMode(addressB, OUTPUT);  //         -5
  pinMode(addressC, OUTPUT);  //Higher Bit-4
   // wake up I2C bus
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
ROLL_c[1] = ROLL[1];
ROLL_c[2] = ROLL[2];
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  restrictedPitch(accX , accY, accZ);
  
#else // Eq. 28 and 29
  unrestrictedPitch(accX , accY , accZ);
 
#endif
  setStartingAngle(roll,pitch);
  
  timer = micros();

 }


void loop()
{
    dt = (double)(micros()-timer) / 1000000; // Calculate delta time
    timer = micros();
    
     tcaselect(MPU1); read_imusKal(1); readImuComp(1);
     tcaselect(MPU2); read_imusKal(2);// readImuComp(2);
     tcaselect(MPU3); read_imusKal(3); //readImuComp(3);
     tcaselect(MPU4); read_imusKal(4); //readImuComp(4);
     tcaselect(MPU5); read_imusKal(5); //readImuComp(5);
     
//Calculate Using Kalman Filter
 /*   v1.getDirectionVector(ROLL[1],PITCH[1],0); v1.GlobalVector(0,90,180);v1.anotherRotation(0,0,0);v1.finalRotation(-ROLL[5],-PITCH[5],0);
     v2.getDirectionVector(ROLL[2],PITCH[2],0);v2.GlobalVector(0,90,180);v2.anotherRotation(0,0,120);v2.finalRotation(-ROLL[5],-PITCH[5],0);
     v3.getDirectionVector(ROLL[3],PITCH[3],0); v3.GlobalVector(0,90,180);v3.anotherRotation(0,0,-120);v3.finalRotation(-ROLL[5],-PITCH[5],0);
     
  /*   Serial.print("Kal");Serial.print(ROLL[1]);Serial.print("\t");Serial.print(PITCH[1]);Serial.print("\t");
     Serial.print(ROLL[2]);Serial.print("\t");Serial.print(PITCH[2]);Serial.print("\t");
     Serial.print(ROLL[3]);Serial.print("\t");Serial.print(PITCH[3]);Serial.print("\t");
     Serial.print(ROLL[4]);Serial.print("\t");Serial.print(PITCH[4]);Serial.print("\t");
     Serial.print(ROLL[5]);Serial.print("\t");Serial.print(PITCH[5]);Serial.print("\t");Serial.println(); 
  
   DK.RotationMatrix(ROLL[4]-ROLL[5],PITCH[4]-PITCH[5]); 
     
    double *vector1 = v1.GetGlobalVector();
    double *vector2 = v2.GetGlobalVector();
    double *vector3 = v3.GetGlobalVector();

  /*  Serial.println("KAL");
       Matrix.Print((double*)vector1, 3, 1, "Printing Vector1"); delay(500);
       Matrix.Print((double*)vector2, 3, 1, "Printing Vector2"); delay(500);
       Matrix.Print((double*)vector3, 3, 1, "Printing Vector3"); delay(500); 

  
    DK.passVectors(vector1 , vector2 , vector3,3);
    DK.constructMatrixC(1);
    DK.getPlatformPosition();
    DK.calculateInverseKinematics();
    getActuatorLengths(); 
      
   //  Serial.print(ROLL[4]-ROLL[5]);Serial.print("\t");Serial.print(PITCH[4]-PITCH[5]);Serial.println();
*/

//Calculate Using Complementary Filter
     v1.getDirectionVector(ROLL_c[1],PITCH_c[1],0); v1.GlobalVector(0,90,180);v1.anotherRotation(0,0,0);v1.finalRotation(180-ROLL[5],-PITCH[5],0);
     v2.getDirectionVector(ROLL_c[2],PITCH_c[2],0);v2.GlobalVector(0,90,180);v2.anotherRotation(0,0,120);v2.finalRotation(180-ROLL[5],-PITCH[5],0);
     v3.getDirectionVector(ROLL_c[3],PITCH_c[3],0); v3.GlobalVector(0,90,180);v3.anotherRotation(0,0,-120);v3.finalRotation(180-ROLL[5],-PITCH[5],0);

 /*   Serial.print("COMP");Serial.print("\t");Serial.print(ROLL_c[1]);Serial.print("\t");Serial.print(PITCH_c[1]);Serial.print("\t");
    Serial.print(ROLL_c[2]);Serial.print("\t");Serial.print(PITCH_c[2]);Serial.print("\t");
    Serial.print(ROLL_c[3]);Serial.print("\t");Serial.print(PITCH_c[3]);Serial.print("\t");Serial.println();
  /*   DK.RotationMatrix(ROLL_c[4]-ROLL_c[5],PITCH_c[4]-PITCH_c[5]);
     
    double *vector1 = v1.GetGlobalVector();
    double *vector2 = v2.GetGlobalVector();
    double *vector3 = v3.GetGlobalVector();
    DK.passVectors(vector1 , vector2 , vector3,3);
    DK.constructMatrixC(1);
    DK.getPlatformPosition();
    DK.calculateInverseKinematics();
    getActuatorLengths(); 
       
  */   
     
//Calculate Using Accelerometer Raw Values

   /*  v1.getDirectionVector(ROLL_raw[1],PITCH_raw[1],0); v1.GlobalVector(0,90,180);v1.anotherRotation(0,0,0);v1.finalRotation(-ROLL[5],-PITCH[5],0);
     v2.getDirectionVector(ROLL_raw[2],PITCH_raw[2],0);v2.GlobalVector(0,90,180);v2.anotherRotation(0,0,120);v2.finalRotation(-ROLL[5],-PITCH[5],0);
     v3.getDirectionVector(ROLL_raw[3],PITCH_raw[3],0); v3.GlobalVector(0,90,180);v3.anotherRotation(0,0,-120);v3.finalRotation(-ROLL[5],-PITCH[5],0);

 /*   Serial.print("RAW");Serial.print("\t");Serial.print(ROLL_raw[1]);Serial.print("\t");Serial.print(PITCH_raw[1]);Serial.print("\t");
    Serial.print(ROLL_raw[2]);Serial.print("\t");Serial.print(PITCH_raw[2]);Serial.print("\t");
    Serial.print(ROLL_raw[3]);Serial.print("\t");Serial.print(PITCH_raw[3]);Serial.print("\t");
    Serial.print(ROLL_raw[4]);Serial.print("\t");Serial.print(PITCH_raw[4]);Serial.print("\t");
    Serial.print(ROLL_raw[5]);Serial.print("\t");Serial.print(PITCH_raw[5]);Serial.print("\t");Serial.println();
    
     DK.RotationMatrix(ROLL_raw[4]-ROLL_raw[5],PITCH_raw[4]-PITCH_raw[5]);

    double *vector1_r = v1.GetGlobalVector();
    double *vector2_r = v2.GetGlobalVector();
    double *vector3_r = v3.GetGlobalVector();

     /*  Serial.println("RAW");
       Matrix.Print((double*)vector1_r, 3, 1, "Printing Vector1"); delay(500);
       Matrix.Print((double*)vector2_r, 3, 1, "Printing Vector2"); delay(500);
       Matrix.Print((double*)vector3_r, 3, 1, "Printing Vector3"); delay(500); 

    
    DK.passVectors(vector1 , vector2 , vector3,3);
    DK.constructMatrixC(1);
    DK.getPlatformPosition();
    DK.calculateInverseKinematics();
    getActuatorLengths(); 
    
       
    /*  Matrix.Print((double*)vector1, 3, 1, "Printing Vector1"); delay(500);
       Matrix.Print((double*)vector2, 3, 1, "Printing Vector2"); delay(500);
       Matrix.Print((double*)vector3, 3, 1, "Printing Vector3"); delay(500); */

    //  Serial.print(ROLL[4]); Serial.print("\t");Serial.print(PITCH[4]);Serial.print("\t");
    // Serial.print(ROLL[5]); Serial.print("\t");Serial.print(PITCH[5]);Serial.print("\t");Serial.println();
   //   Serial.print(ROLL[4]-ROLL[5]); Serial.print("\t");Serial.print(PITCH[4]-PITCH[5]);Serial.println();
    
        
      
       
/*Serial.print("Kalman"); Serial.print("\t");Serial.print(ROLL[1]); Serial.print("\t");Serial.print(PITCH[1]); Serial.print("\t");
Serial.print("Comp"); Serial.print("\t");Serial.print(ROLL_c[1]); Serial.print("\t");Serial.print(PITCH_c[1]); Serial.print("\t");
Serial.print("RAW"); Serial.print("\t");Serial.print(ROLL_raw[1]); Serial.print("\t");Serial.print(PITCH_raw[1]); Serial.println();
*/
/*Serial.print("Acc"); 
Serial.print(ax[1]);Serial.print("\t");Serial.print(ay[1]);Serial.print("\t");Serial.print(az[1]);Serial.print("\t");//Serial.println();delay(1000);
Serial.print("Acc"); 
Serial.print(ax[2]);Serial.print("\t");Serial.print(ay[2]);Serial.print("\t");Serial.print(az[2]);Serial.print("\t");//Serial.println();delay(1000);
Serial.print("Acc"); 
Serial.print(ax[3]);Serial.print("\t");Serial.print(ay[3]);Serial.print("\t");Serial.print(az[3]);Serial.print("\t");//Serial.println();delay(1000);
Serial.print("Acc"); 
Serial.print(ax[4]);Serial.print("\t");Serial.print(ay[4]);Serial.print("\t");Serial.print(az[4]);
Serial.print("Acc"); 
Serial.print(ax[5]);Serial.print("\t");Serial.print(ay[5]);Serial.print("\t");Serial.print(az[5]);
Serial.println();delay(1000); */
  //Serial.print("Gyro"); Serial.print(gx[5]);Serial.print(gy[5]);Serial.print(gz[5]); Serial.println();delay(1000); 
 // Serial.print("Execution Time");Serial.print("\t");Serial.print(dt);Serial.println();
  

}

void read_imusKal(int a)
{
  int i = a;
  
  accelGyroMag.initialize();
  accelGyroMag.getMotion6(&AX, &AY, &AZ, &GX, &GY, &GZ);
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  
  ax[i] = (double) AX;  ay[i] = (double) AY;  az[i] = (double) AZ;
  gx[i]= (double) GX;  gy[i] = (double) GY;  gz[i] = (double) GZ;
  accX = ax[i] ; accY = ay[i] ; accZ = az[i] ;
  gyroX = gx[i]; gyroY = gy[i]; gyroZ = gz[i]; 

  //Calculate Roll and Pitch using Raw Accelerometer Values

 


   
    
 
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
 
 ROLL_c[i] = compAngleX; PITCH_c[i] = compAngleY ;

 ROLL_raw[i] = atan(ay[i]/sqrt(ax[i] * ax[i] + az[i] * az[i])) * RAD_TO_DEG;
 PITCH_raw[i] = atan(-ax[i] / sqrt(ay[i] * ay[i] + az[i] * az[i])) * RAD_TO_DEG;
 
 
  }




  double restrictedPitch(double x ,double y, double z)
  {
   accX = x; accY = y; accZ= z;
   
  //Roll RANGE FROM -180 TO +180
  /* roll  = atan2(accY, accZ) * RAD_TO_DEG;
   pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; */

//ROLL RANGE FROM -90 TO +90
   roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
   pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
   return roll , pitch;
   }

double unrestrictedPitch(double a, double b, double c)
{
  accX = a; accY = b; accZ= c;
  //PITCH RANGE FROM -180 TO 180
 /*
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
  */
  //PITCH RANGE FROM -90 TO +90
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
  
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
    //double *ErrorCovarianceX = kalmanX.getErrorCovariance();
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt /*ErrorCovarianceX*/); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
  {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt /*ErrorCovarianceY*/);
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
   // double *ErrorCovarianceY = kalmanY.getErrorCovariance();
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt/*ErrorCovarianceY*/); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
  {
  gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  // double *ErrorCovarianceX = kalmanX.getErrorCovariance();
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt/* ErrorCovarianceX*/); // Calculate the angle using a Kalman filter
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
//Serial.print(compAngleX) ; Serial.print("\t"); Serial.print(compAngleY); Serial.println();
  // Reset the gyro angle when it has drifted too much
    gyroReset(gyroXangle , gyroYangle);
    return compAngleX , compAngleY;
  }

  double convertGyroRate(double xRate , double yRate)
  {
  gyroXrate = xRate; gyroYrate = yRate;
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  }

void tcaselect(uint8_t i) 
{
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
  //Serial.print(i);
}

void getActuatorLengths()
{
    int dead_length = 183;//(168 - Length Of Unactuated Actuator + 15- Connection between Actuator and Joint)
   Fb1_val = analogRead(Fb_M1);
   len1 = Fb1_val*(voltage2mm) + dead_length;
   Fb2_val = analogRead(Fb_M2); 
    len2 = Fb2_val*(voltage2mm) + dead_length;
    Fb3_val = analogRead(Fb_M3); 
    len3 = Fb3_val*(voltage2mm) + dead_length;
  // Serial.println("Printing Lengths from Actuator Reading");
    // Serial.print(Fb1_val);Serial.print('\t');Serial.print(Fb2_val);Serial.print('\t');Serial.print(Fb3_val);Serial.println();
    Serial.print(len1,3);Serial.print('\t');Serial.print(len2,3);Serial.print('\t');Serial.print(len3,3);Serial.println();
}

double readImuComp(int a)
{
 Serial.print("Initial ROll_c"); Serial.print("\t");Serial.print(ROLL_c[i]);Serial.println();
  i = a;
   gyroXrate_comp[i] = gx[i] / 131.0; // Convert to deg/s
   gyroYrate_comp[i] = gy[i] / 131.0; // Convert to deg/s
  Serial.print("Gyro_rate"); Serial.print(gyroXrate_comp[i]); Serial.print("\t");Serial.print(gyroYrate_comp[i]);Serial.println();
  Serial.print("dt");Serial.print(dt);Serial.println();
 
  ROLL_c[i] = 0.93 * (ROLL_c[i] + gyroXrate_comp[i] * dt) + (0.07 * ROLL[i]); // Calculate the angle using a Complimentary filter
  PITCH_c[i] = 0.93 * (PITCH_c[i] + gyroYrate_comp[i] * dt) + (0.07 * PITCH[i]);
Serial.print("ROLL");Serial.print(ROLL[i]);Serial.print("\t");Serial.print(ROLL_c[i]);Serial.println();
return ROLL_c[i] , PITCH_c[i];
}


