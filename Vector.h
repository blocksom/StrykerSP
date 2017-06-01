#ifndef vect
#define vect

#if (ARDUINO >=100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Vector {
  public:
    float Vector_Dot_Product(float* vector1,float* vector2);
    void Vector_Cross_Product(float* v1,float* v2,float* vectorOut_Cross);
    void Vector_Add(float* vectorIn1, float* vectorIn2,float* vectorOut_Add);
    void Vector_Norm(float* v3,float* vectorOut_Norm); 
    float Norm_const(float v3[3]);
    void Create_SSC(float* vIn,float sscOut[3][3]);
    void Identity(float identity[3][3]);
    void Add(float* A, float* B, int m, int n, float* C);
    void Matrix_Square(float array3[3][3],float squared_matrix[3][3]);
    void Vector_Square(float* v1,float* squared_vector);
    void Matrix_Const_Mult(float array1[3][3], float constant,float matrix_const_mult[3][3]);
    void Multiply(float* A, float* B, int m, int p, int n, float* C);
    void Transpose(float* A, int m, int n, float* C);
    void Print(float* A, int m, int n, String label);
    float Average(float* array4, int s);
    int Invert(float* A, int n);
    void SumElements(float dataIn[3],int N,float dataOut);
};

#endif
