/*
MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)
Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/
MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/
sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/
MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.
MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.
You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.
*/

//look into eigen library

#include "Vector.h"

//Computes the dot product of two vectors
float Vector::Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;
  
  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

//Computes the cross product of two vectors
void Vector::Vector_Cross_Product(float* v1,float* v2,float* vectorOut_Cross)
{
  vectorOut_Cross[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut_Cross[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut_Cross[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar. 
//void Vector::Vector_Scale(float* vectorOut,float* vectorIn, float* scale2,float* vectorOut_Scalar)
//{
//  for(int c=0; c<3; c++)
//  {
//   vectorOut_Scalar[c]=vectorIn[c]*scale2; 
//  }
//}

//Addition of vectors
void Vector::Vector_Add(float* vectorIn1, float* vectorIn2,float* vectorOut_Add)
{
  for(int c=0; c<3; c++)
  {
     vectorOut_Add[c]=vectorIn1[c]+vectorIn2[c];
  }

}

//Normalize a vector
void Vector::Vector_Norm(float* v3,float* vectorOut_Norm)
{
  float L = sqrt((v3[0] * v3[0]) + (v3[1] * v3[1]) + (v3[2] * v3[2])); 
  vectorOut_Norm[0] = v3[0]/L;
  vectorOut_Norm[1] = v3[1]/L;
  vectorOut_Norm[2] = v3[2]/L;
  
}

// Create normalization constant
float Vector::Norm_const(float v3[3])
{
  float L = sqrt((v3[0] * v3[0]) + (v3[1] * v3[1]) + (v3[2] * v3[2])); 
  return L;
}

// Create SSC Array

void Vector::Create_SSC(float* vIn,float sscOut[3][3]) {
  sscOut[0][0] = { 0 };
  sscOut[0][1] = { -1 * vIn[2] };
  sscOut[0][2] = { vIn[1] };
  sscOut[1][0] = { vIn[2] };
  sscOut[1][1] = { 0 };
  sscOut[1][2] = { -1 * vIn[0] };
  sscOut[2][0] = { -1 * vIn[1] };
  sscOut[2][1] = { vIn[0] };
  sscOut[2][2] = { 0 };
}

// Create Identity

void Vector::Identity(float identity[3][3]){
  identity[0][0] = { 1 };
  identity[0][1] = { 0 };
  identity[0][2] = { 0 };
  identity[1][0] = { 0 };
  identity[1][1] = { 1 };
  identity[1][2] = { 0 };
  identity[2][0] = { 0 };
  identity[2][1] = { 0 };
  identity[2][2] = { 1 };
}

//Matrix Addition Routine
void Vector::Add(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A+B (m x n)
  int i, j;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
      C[n*i+j]=A[n*i+j]+B[n*i+j];
}

// Squaring Matrix

void Vector::Matrix_Square(float array3[3][3], float squared_matrix[3][3]){
  for (int i=0; i<3; i++){
    for (int j=0;j<3;j++){
      squared_matrix[i][j] = (pow(array3[i][j],2)); 
    }    
  }
}

// Squaring Vector

void Vector::Vector_Square(float* v1,float* squared_vector){
  for (int i=0;i<3;i++){
    squared_vector[i] = pow(v1[i],2); // look into power function for arduino if same as cpp
  }
}

// Multiplying matrix by constant

void Vector::Matrix_Const_Mult(float array1[3][3] , float constant, float matrix_const_mult[3][3]){
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
    matrix_const_mult[i][j] = constant*array1[i][j];
    }
  }
}

//Matrix Multiplication Routine
// C = A*B
void Vector::Multiply(float* A, float* B, int m, int p, int n, float* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
    {
      C[n*i+j]=0;
      for (k=0;k<p;k++)
        C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
    }
}

//Matrix Transpose Routine
void Vector::Transpose(float* A, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)
  int i, j;
  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
      C[m*j+i]=A[n*i+j];
}

// Average of a Matrix

float Vector::Average(float* array4, int s)
{
      //int s=20;              //Array size
      //int array[m];          //Declaring array
      float sum=0;                
      for(int i=0;i<s;i++)   //Loop which inputs arrays data and
                                //Calculates its sum
      {
              //cout<<"Enter element number "<<i+1<<endl;
              array4[i];
              sum=sum+array4[i];
      }
      float Ave;
      Ave = sum/s;
      return Ave;
      //Now calling division function to find the average...
      //cout<<"Average of array elements is "<<division(sum,size);
      
}

// Matrix Printing Routine
// Uses tabs to separate numbers under assumption printed float width won't cause problems
void Vector::Print(float* A, int m, int n, String label){
  // A = input matrix (m x n)
  int i,j;
  Serial.println();
  Serial.println(label);
  for (i=0; i<m; i++){
    for (j=0;j<n;j++){
      Serial.print(A[n*i+j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//   NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int Vector::Invert(float* A, int n)
{
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow;     // keeps track of current pivot row
    int k,i,j;      // k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    float tmp;      // used for finding max value and making column swaps
 
    for (k = 0; k < n; k++)
    {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < n; i++)
        {
            if (abs(A[i*n+k]) >= tmp)   // 'Avoid using other functions inside abs()?'
            {
                tmp = abs(A[i*n+k]);
                pivrow = i;
            }
        }
 
        // check for singular matrix
        if (A[pivrow*n+k] == 0.0f)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }
 
        // Execute pivot (row swap) if needed
        if (pivrow != k)
        {
            // swap row k with pivrow
            for (j = 0; j < n; j++)
            {
                tmp = A[k*n+j];
                A[k*n+j] = A[pivrow*n+j];
                A[pivrow*n+j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)
 
        tmp = 1.0f/A[k*n+k];    // invert pivot element
        A[k*n+k] = 1.0f;        // This element of input matrix becomes result matrix
 
        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++)
        {
            A[k*n+j] = A[k*n+j]*tmp;
        }
 
        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                tmp = A[i*n+k];
                A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++)
                {
                    A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
                }
            }
        }
    }
 
    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                tmp = A[i*n+k];
                A[i*n+k] = A[i*n+pivrows[k]];
                A[i*n+pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}

// Addition of elements of array

void Vector::SumElements(float dataIn[3],int N, float dataOut){;
dataOut = 0;
int i;
      for (i=0; i<N; i++){
     
        dataOut=  dataOut+dataIn[i];
      }
}

