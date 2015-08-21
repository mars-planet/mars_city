/*
Copyright 2010. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef __ELIMINATION_H__
#define __ELIMINATION_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

class Elimination {
  
public:
  
  Elimination (int32_t m,int32_t n) : m(m),n(n) {
    if (m>0 && n>0) {
      A = allocateMatrix(m,m);
      B = allocateMatrix(m,n);
    }
  }
  
  ~Elimination () {
    if (m>0 && n>0) {
      freeMatrix(A);
      freeMatrix(B);
    }
  }
  
  bool gaussJordan () {
    return gaussJordanElimination(A,m,B,n);
  }
  
  void reset () {
    zeroMatrix(A,m,m);
    zeroMatrix(B,m,n);
  }
  
  void printA () {
    printMatrix(A,m,m);
  }
  
  void printb () {
    printMatrix(B,m,n);
  }

  int32_t m,n;
  float** A;
  float** B;
  
private:
  
  float** allocateMatrix(int32_t nrow,int32_t ncol);
  void freeMatrix(float** mat);
  void zeroMatrix(float** mat, int32_t nrow,int32_t ncol);
  void printMatrix(float** mat, int32_t nrow,int32_t ncol);
  bool gaussJordanElimination(float **A, int m, float **B, int n, float eps=1e-8);
};

#endif
