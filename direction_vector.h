#include <MatrixMath.h>
#ifndef _direction_vector_h
#define  _direction_vector_h
#define N (3)

//double v_tilda[N][N];
double D[N][1]; //Direction Vector to define local frame
//double AB[N][N];
double ABC[N][N];
double dir_vec[N][1]; //Global Direction Vector
double local_vec[N][1]; 



class direction_vector
{
  public:
  direction_vector()
  {
   /* A[N][N]=0;
    B[N][N]=0; 
    C[N][N]=0;
    D[1][N]=0;
    AB[N][N]=0;
    ABC[N][N]=0;
    ROT[1][N]=0; //Direction Vector
    norm=0;
   v1_tilda[N][N]=0;
    */
  };

void getDirectionVector(double r , double p , double y )
{
  roll = r*(PI/180); pitch=p*(PI/180) ; yaw=y*(PI/180);
  
  //Initializing the Rotation Matrices
/*  A[0][0] = 1; A[0][1] = 0;         A[0][2] = 0;
  A[1][0] = 0; A[1][1] = cos(roll); A[1][2] = -sin(roll);
  A[2][0] = 0; A[2][1] = sin(roll); A[2][2] = cos(roll);

  B[0][0] = cos(pitch); B[0][1] = 0; B[0][2] = sin(pitch);
  B[1][0] = 0;          B[1][1] = 1; B[1][2] = 0;
  B[2][0] = -sin(pitch);B[2][1] = 0; B[2][2] = cos(pitch);

  C[0][0] = cos(yaw); C[0][1] = -sin(yaw); C[0][2] = 0;
  C[1][0] = sin(yaw); C[1][1] = cos(yaw);  C[1][2] = 0;
  C[2][0] = 0;         C[2][1] = 0;          C[2][2] = 1;

 // C[0][0] = 1; C[0][1] = 0; C[0][2] = 0;
  //C[1][0] = 0; C[1][1] = 1;  C[1][2] = 0;
  //C[2][0] = 0;         C[2][1] = 0;          C[2][2] = 1;
  */
  ABC[0][0] = cos(pitch)*cos(yaw) ;                                   ABC[0][1] = -cos(pitch)*sin(yaw);                             ABC[0][2] = sin(pitch) ;
  ABC[1][0] = cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw) ;      ABC[1][1] = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw); ABC[1][2] = -sin(roll)*cos(pitch) ;
  ABC[2][0] = sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw)  ;     ABC[2][1] = sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw); ABC[2][2] = cos(roll)*cos(pitch) ;
 
  D[0][0] = 0;
  D[1][0] = 0;
  D[2][0] = 1;
//Matrix multiplication
// Rot_matrix = Rot_matrix(roll)*Rot_matrix(pitch)*Rot_matrix(yaw)
// Direction_Vec = Rot_matrix*dir_vec(i.e Z axis)
//Matrix.Multiply((double*)A, (double*)B, N, N, N, (double*)AB);
  //Matrix.Multiply((double*)AB, (double*)C, N, N, N, (double*)ABC);
  
};

  void GlobalVector(double a,double b,double c)
  {
    x=a; y=b; z=c;
    Matrix.Multiply((double*)ABC, (double*)D, N, N, 1, (double*)local_vec);
    //Matrix.Print((double*)local_vec, N, 1, "Local Vector");
    getDirectionVector(x,y,z);
    Matrix.Multiply((double*)ABC, (double*)local_vec, N, N, 1, (double*)dir_vec);
   // Matrix.Print((double*)dir_vec, N, 1, "Direction Vector");
    
  };

 
   void anotherRotation(double a,double b,double c)
  {
    x=a; y=b; z=c;
    getDirectionVector(x,y,z);
    Matrix.Multiply((double*)ABC, (double*)dir_vec, N, N, 1, (double*)global_dir);
    //Matrix.Print((double*)global_dir, N, 1, "After Another rotation");
    
   };

   double* GetGlobalVector()
   {
    return *global_dir;
   };
private :
//double A[N][N]; //Rotation about X axis
//double B[N][N]; //Rotation abour Y axis
//double C[N][N]; //Rotation about Z axis (Identity Matrix)

double roll,pitch,yaw,x,y,z;
double norm;
double norm_global;
double global_dir[N][1];
  



};

#endif








