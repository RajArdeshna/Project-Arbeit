#ifndef _DirectKinematics_h
#define _DirectKinematics_h
#include <MatrixMath.h>
#define N (3)

  
 
  
  double Rb1[N] , Rb2[N] , Rb3[N] ;
  double d1[N],d2[N],d3[N];
  double Rho[N];
  int select_filter;
  double Rba1[N] , Rba2[N] , Rba3[N] ;
  double c1[N] ,c2[N] ,c3[N] ;
  
  double A_trans[3*N][N];
  double A_transC[N];
  double A_transA[N][N];
  double A_transA_inv[N][N];
  double C[3*N];
  double a1[N] , a2[N] , a3[N]; //Position Vectors of Revolute Joint wrt Main Frame
  double b1[N] , b2[N] , b3[N]; //Position Vectors of Spherical Joint wrt Manipulator Frame
  double Plat_Pos[N];
  double Base_radius=120 , Manipulator_radius=78;
  double alpha , beta , gamma  ;
  double a_A ,b_A ,g_A;
 
  
  

class DirectKinematics
{
  public:

  DirectKinematics()
  {
    //Base_radius = 130; Manipulator_radius = 70;
    
    a1[0] = Base_radius;      a1[1] = 0;                          a1[2] = 8;
    a2[0] = -(Base_radius/2); a2[1] = Base_radius*((sqrt(3))/2);  a2[2] = 8;
    a3[0] = -(Base_radius/2);  a3[1] = -Base_radius*((sqrt(3))/2); a3[2] = 8; 

    b1[0] = Manipulator_radius;       b1[1]= 0;                                 b1[2]=-10.4;
    b2[0] = -(Manipulator_radius/2);  b2[1]= Manipulator_radius*((sqrt(3))/2);  b2[2]=-10.4;
    b3[0] =-(Manipulator_radius/2);    b3[1]= -Manipulator_radius*((sqrt(3))/2); b3[2]=-10.4;
    
  };
  
  void passVectors(double vec1[] ,double vec2[] , double vec3[],int n)
  {
    for(int i = 0; i < n; i++)
    {
      dk_v1[i]= vec1[i]; dk_v2[i]= vec2[i]; dk_v3[i]= vec3[i];
    }

 v1_tilda[0][0]=A[0][0] = 0 ;        v1_tilda[0][1]=A[0][1] = -dk_v1[2];    v1_tilda[0][2]=A[0][2] = dk_v1[1];
 v1_tilda[1][0]=A[1][0] = dk_v1[2];  v1_tilda[1][1]=A[1][1] = 0;            v1_tilda[1][2]=A[1][2] = -dk_v1[0];
 v1_tilda[2][0]=A[2][0] = -dk_v1[1]; v1_tilda[2][1]=A[2][1] = dk_v1[0];     v1_tilda[2][2]=A[2][2] = 0;

 v2_tilda[0][0]=A[3][0] = 0 ;        v2_tilda[0][1]=A[3][1] = -dk_v2[2];    v2_tilda[0][2]=A[3][2] = dk_v2[1];
 v2_tilda[1][0]=A[4][0] = dk_v2[2];  v2_tilda[1][1]=A[4][1] = 0;            v2_tilda[1][2]=A[4][2] = -dk_v2[0];
 v2_tilda[2][0]=A[5][0] = -dk_v2[1]; v2_tilda[2][1]=A[5][1] = dk_v2[0];     v2_tilda[2][2]=A[5][2] = 0;

 v3_tilda[0][0]=A[6][0] = 0 ;        v3_tilda[0][1]=A[6][1] = -dk_v3[2];    v3_tilda[0][2]=A[6][2] = dk_v3[1];
 v3_tilda[1][0]=A[7][0] = dk_v3[2];  v3_tilda[1][1]=A[7][1] = 0;            v3_tilda[1][2]=A[7][2] = -dk_v3[0];
 v3_tilda[2][0]=A[8][0] = -dk_v3[1]; v3_tilda[2][1]=A[8][1] = dk_v3[0];     v3_tilda[2][2]=A[8][2] = 0;

    //Matrix.Print((double*)dk_v1, N,1, "dk_v1");
    //Matrix.Print((double*)dk_v2, N,1, "dk_v2");
    //Matrix.Print((double*)dk_v3, N,1, "dk_v3");
  // Matrix.Print((double*)v1_tilda, N, N, "v1_tilda");
  // Matrix.Print((double*)v2_tilda, N, N, "v2_tilda");
  // Matrix.Print((double*)v3_tilda, N, N, "v3_tilda");
//  Matrix.Print((double*)A, 3*N, N, "A");
    
  };

  void ROT(double alpha_A , double beta_A , double gamma_A)
  {
    a_A = PI/180*alpha_A;
    b_A = PI/180*beta_A;
    g_A = PI/180*gamma_A;

        
 R[0][0] = cos(b_A)*cos(g_A) ;                             R[0][1] = -cos(b_A)*sin(g_A);                          R[0][2] = sin(b_A) ;
 R[1][0] = cos(a_A)*sin(g_A)+sin(a_A)*sin(b_A)*cos(g_A) ;  R[1][1] = cos(a_A)*cos(g_A)-sin(a_A)*sin(b_A)*sin(g_A);R[1][2] = -sin(a_A)*cos(b_A) ;
 R[2][0] = sin(a_A)*sin(g_A)-cos(a_A)*sin(b_A)*cos(g_A)  ; R[2][1] = sin(a_A)*cos(g_A)+cos(a_A)*sin(b_A)*sin(g_A);R[2][2] = cos(a_A)*cos(b_A) ;

    
  };

  void RotationMatrix(double roll , double pitch)
  {
    alpha = roll ; beta = pitch;
    gamma = -atan2(sin(alpha*PI/180)*sin(beta*PI/180),cos(alpha*PI/180)+cos(beta*PI/180))*180/PI;
    //Serial.print(gamma);
    //Serial.print("Alpha");
   // Serial.print("\t");Serial.print(alpha);Serial.print("\t");
//   // Serial.print("Beta");
   // Serial.print("\t");Serial.print(beta);Serial.print("\t");
//  //  Serial.print("Gamma");
 //  Serial.print("\t");Serial.print(gamma);Serial.print("\t");
    ROT(alpha , beta , gamma);
 // Matrix.Print((double*)R, N, N, "Rotation Matrix");

  };

  void constructMatrixC(int filter)
  {
     select_filter = filter;
     
     Matrix.Multiply((double*)R, (double*)b1, N, N, 1, (double*)Rb1);
     Matrix.Multiply((double*)R, (double*)b2, N, N, 1, (double*)Rb2);
     Matrix.Multiply((double*)R, (double*)b3, N, N, 1, (double*)Rb3);

         
     Matrix.Subtract((double*)a1, (double*)Rb1, N, 1, (double*)Rba1);
     Matrix.Subtract((double*)a2, (double*)Rb2, N, 1, (double*)Rba2);
     Matrix.Subtract((double*)a3, (double*)Rb3, N, 1, (double*)Rba3);

     Matrix.Multiply((double*)v1_tilda, (double*)Rba1, N, N, 1, (double*)c1);
     Matrix.Multiply((double*)v2_tilda, (double*)Rba2, N, N, 1, (double*)c2);
     Matrix.Multiply((double*)v3_tilda, (double*)Rba3, N, N, 1, (double*)c3);

     if (select_filter == 1)
     {
     C[0] = -c1[0];  
     C[1] = -c1[1];
     C[2] = -c1[2];
     C[3] = -c2[0];
     C[4] = -c2[1];
     C[5] = -c2[2];
     C[6] = -c3[0];
     C[7] = -c3[1];
     C[8] = -c3[2];
     }

if(select_filter == 3)
{
     C[0] = c1[0];  
     C[1] = c1[1];
     C[2] = c1[2];
     C[3] = c2[0];
     C[4] = c2[1];
     C[5] = c2[2];
     C[6] = c3[0];
     C[7] = c3[1];
     C[8] = c3[2];
}
   // Matrix.Print((double*)C, 3*N, 1, "Matrix C");
     
  };

    void getPlatformPosition()
  {
   // Matrix.Print((double*)A, 3*N, 3, "A");
    Matrix.Transpose((double*)A, 3*N, 3, (double*) A_trans);
   // Matrix.Print((double*)A_trans, N, 3*N, "A_trans");
    Matrix.Multiply((double*)A_trans, (double*)C, N, 3*N, 1, (double*)A_transC);
   // Matrix.Print((double*)A_transC, N, 1, "A_transC");
    Matrix.Multiply((double*)A_trans, (double*)A, N, 3*N, N, (double*)A_transA);
    //Matrix.Print((double*)A_transA, N, N, "A_transA");
    Matrix.Invert((double*)A_transA , N );
   // Matrix.Print((double*)A_transA, N, N, "A_transA_inv");
    Matrix.Multiply((double*)A_transA, (double*)A_transC, N, N, 1, (double*)Plat_Pos);
 // Serial.println("Platform pose");
   Serial.print(Plat_Pos[0],3);Serial.print("\t");Serial.print(Plat_Pos[1],3);Serial.print("\t");Serial.print(Plat_Pos[2],3); Serial.print("\t");
   Serial.print("\t");Serial.print(alpha,3);Serial.print("\t");
   Serial.print("\t");Serial.print(beta,3);Serial.print("\t");
   Serial.print("\t");Serial.print(gamma,3);Serial.print("\t"); //Serial.println();
   //  RR[9] = Plat_Pos[0];  RR[10] = Plat_Pos[1];   RR[11] = Plat_Pos[2]; */
    
  };

 void calculateInverseKinematics()
 {
   for(int i=0;i<3;i++)
  {
  d1[i] = Plat_Pos[i] + Rb1[i] - a1[i];
  d2[i] = Plat_Pos[i] + Rb2[i] - a2[i];
  d3[i] = Plat_Pos[i] + Rb3[i] - a3[i];
  }

   Rho[0] = sqrt(d1[0]*d1[0] + d1[1]*d1[1] + d1[2]*d1[2]);
   Rho[1] = sqrt(d2[0]*d2[0] + d2[1]*d2[1] + d2[2]*d2[2]);
   Rho[2] = sqrt(d3[0]*d3[0] + d3[1]*d3[1] + d3[2]*d3[2]); 
 // Serial.println("Readings from Inverse Kinematics");
 Serial.print(Rho[0],3);Serial.print('\t');Serial.print(Rho[1],3);Serial.print('\t');Serial.print(Rho[2],3);Serial.print("\t");

  
 };

  private :
  double v1_tilda[N][N];
  double v2_tilda[N][N];
  double v3_tilda[N][N];
  double dk_v1[N];
  double dk_v2[N];
  double dk_v3[N];
  double R[N][N];
  double A[3*N][N];
 
    
 
};





#endif

