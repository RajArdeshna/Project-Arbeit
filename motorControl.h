#include <MatrixMath.h>
#include 
#ifndef _motorControl_h
#define _motorControl_h
#define N (3)

class motorControl
{
  public:

   motorControl()
  {
    a1[0] = Base_radius;      a1[1] = 0;                          a1[2] = 0;
    a2[0] = -(Base_radius/2); a2[1] = Base_radius*((sqrt(3))/2);  a2[2] = 0;
    a3[0] = -(Base_radius/2);  a3[1] = -Base_radius*((sqrt(3))/2); a3[2] = 0; 

    b1[0] = Manipulator_radius;       b1[1]= 0;                                 b1[2]=0;
    b2[0] = -(Manipulator_radius/2);  b2[1]= Manipulator_radius*((sqrt(3))/2);  b2[2]=0;
    b3[0] =-(Manipulator_radius/2);    b3[1]= -Manipulator_radius*((sqrt(3))/2); b3[2]=0;
    
  };

 void M_ROT(double alpha_A , double beta_A , double gamma_A)
  {
    a_A = PI/180*alpha_A;
    b_A = PI/180*beta_A;
    g_A = PI/180*gamma_A;

        
 M_R[0][0] = cos(b_A)*cos(g_A) ;                             M_R[0][1] = -cos(b_A)*sin(g_A);                          M_R[0][2] = sin(b_A) ;
 M_R[1][0] = cos(a_A)*sin(g_A)+sin(a_A)*sin(b_A)*cos(g_A) ;  M_R[1][1] = cos(a_A)*cos(g_A)-sin(a_A)*sin(b_A)*sin(g_A);M_R[1][2] = -sin(a_A)*cos(b_A) ;
 M_R[2][0] = sin(a_A)*sin(g_A)-cos(a_A)*sin(b_A)*cos(g_A)  ; M_R[2][1] = sin(a_A)*cos(g_A)+cos(a_A)*sin(b_A)*sin(g_A);M_R[2][2] = cos(a_A)*cos(b_A) ;
    
  };

   void Validate(double roll , double pitch , double len)
  {
    alpha = roll ; beta = pitch; z=len;
    gamma = -atan2(sin(alpha*PI/180)*sin(beta*PI/180),cos(alpha*PI/180)+cos(beta*PI/180))*180/PI;
    M_ROT(alpha , beta , gamma);
    
    p[0]=Manipulator_radius*(sqrt(3)*R[1,1]-sqrt(3)*R[2,2]-3*R[2,1]+3*R[1,2])*sqrt(3)/6;   
    p[1]=-Manipulator_radius*R[2,1]; 
    p[2]= z;

     Matrix.Multiply((double*)M_R, (double*)b1, N, N, 1, (double*)M_Rb1);
     Matrix.Multiply((double*)M_R, (double*)b2, N, N, 1, (double*)M_Rb2);
     Matrix.Multiply((double*)M_R, (double*)b3, N, N, 1, (double*)M_Rb3);
   
   for(int i=0;i<3;i++)
  {
  d1[i] = p[i] + M_Rb1[i] - a1[i];
  d2[i] = p[i] + M_Rb2[i] - a2[i];
  d3[i] = p[i] + M_Rb3[i] - a3[i];
  }

   M_Rho[0] = sqrt(d1[0]*d1[0] + d1[1]*d1[1] + d1[2]*d1[2]);
   M_Rho[1] = sqrt(d2[0]*d2[0] + d2[1]*d2[1] + d2[2]*d2[2]);
   M_Rho[2] = sqrt(d3[0]*d3[0] + d3[1]*d3[1] + d3[2]*d3[2]); 

  // Serial.println("Readings from Validation");
  // Serial.print(M_Rho[0]);Serial.print('\t');Serial.print(M_Rho[1]);Serial.print('\t');Serial.print(M_Rho[2]);Serial.println();


  };

  private:
  double Base_radius=130 , Manipulator_radius=70;
  double p[N];
  double alpha ,beta,gamma,z;
  double M_R[N][N],M_Rb1[N],M_Rb2[N],M_Rb3[N];
  double a1[N],a2[N],a3[M],b1[N],b2[N],b3[N],d1[N],d2[N],d3[N],M_Rho[N];
};

#endif
