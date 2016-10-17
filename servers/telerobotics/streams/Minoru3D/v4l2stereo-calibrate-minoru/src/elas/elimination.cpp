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

#include "elimination.h"

#define SWAP(a,b) {temp=a;a=b;b=temp;}

using namespace std;

float** Elimination::allocateMatrix(int32_t nrow,int32_t ncol) {
  float **mat;
  mat    = (float**)malloc(nrow*sizeof(float*));
  mat[0] = (float*)calloc(nrow*ncol,sizeof(float));
  for(int32_t i=1; i<nrow; i++) mat[i]=mat[i-1]+ncol;
  return mat;
}

void Elimination::freeMatrix(float **mat) {
  free(mat[0]);
  free(mat);
}

void Elimination::zeroMatrix(float** mat, int32_t nrow,int32_t ncol) {
  for (int32_t i=0; i<nrow; i++)
    for (int32_t j=0; j<ncol; j++)
      mat[i][j] = 0;
}

void Elimination::printMatrix(float** mat, int32_t nrow,int32_t ncol) {
  for (int32_t i=0; i<nrow; i++) {
    for (int32_t j=0; j<ncol; j++)
      cout << mat[i][j] << " ";
    cout << endl;
  }
}

// Adopted by "Numerical Recipies in C"
// Linear equation solution by Gauss-Jordan elimination, equation (2.1.1) above. a[1..n][1..n]
// is the input matrix. b[1..n][1..m] is input containing the m right-hand side vectors. On
// output, a is replaced by its matrix inverse, and b is replaced by the corresponding set
// of solution vectors.
bool Elimination::gaussJordanElimination(float **A, int m, float **B, int n, float eps) {
  
  // index vectors for bookkeeping on the pivoting
  int32_t* indxc = new int32_t[m];
  int32_t* indxr = new int32_t[m];
  int32_t* ipiv  = new int32_t[m];
  
  // loop variables
  int32_t i,icol=0,irow=0,j,k,l,ll;
  float big,dum,pivinv,temp; 
  
  // initialize pivots to zero
  for (j=0;j<m;j++) ipiv[j]=0;
  
  // main loop over the columns to be reduced
  for (i=0;i<m;i++) {
    
    big=0.0;
    
    // search for a pivot element
    for (j=0;j<m;j++) 
      if (ipiv[j]!=1)
        for (k=0;k<m;k++)
          if (ipiv[k]==0)
            if (fabs(A[j][k])>=big) {
              big=fabs(A[j][k]);
              irow=j;
              icol=k;
            }
    ++(ipiv[icol]);

    // We now have the pivot element, so we interchange rows, if needed, to put the pivot
    // element on the diagonal. The columns are not physically interchanged, only relabeled:
    // indxc[i], the column of the ith pivot element, is the ith column that is reduced, while
    // indxr[i] is the row in which that pivot element was originally located. If indxr[i] !=
    // indxc[i] there is an implied column interchange. With this form of bookkeeping, the
    // solution b’s will end up in the correct order, and the inverse matrix will be scrambled
    // by columns.
    if (irow != icol) {
      for (l=0;l<m;l++) SWAP(A[irow][l],A[icol][l])
      for (l=0;l<n;l++) SWAP(B[irow][l],B[icol][l])
    }
        
    indxr[i]=irow; // We are now ready to divide the pivot row by the
    indxc[i]=icol; // pivot element, located at irow and icol.
        
    // check for singularity
    if (fabs(A[icol][icol]) < eps) {
      free(indxc);
      free(indxr);
      free(ipiv);
      return false;
    }
    
    pivinv=1.0/A[icol][icol];
    A[icol][icol]=1.0;
    for (l=0;l<m;l++) A[icol][l] *= pivinv;
    for (l=0;l<n;l++) B[icol][l] *= pivinv;
    
    // Next, we reduce the rows except for the pivot one
    for (ll=0;ll<m;ll++) 
    if (ll!=icol) {
      dum = A[ll][icol];
      A[ll][icol] = 0.0;
      for (l=0;l<m;l++) A[ll][l] -= A[icol][l]*dum;
      for (l=0;l<n;l++) B[ll][l] -= B[icol][l]*dum;
    }
  }
  
  // This is the end of the main loop over columns of the reduction. It only remains to unscramble
  // the solution in view of the column interchanges. We do this by interchanging pairs of
  // columns in the reverse order that the permutation was built up.
  for (l=m-1;l>=0;l--) {
    if (indxr[l]!=indxc[l])
      for (k=0;k<m;k++)
        SWAP(A[k][indxr[l]],A[k][indxc[l]])
  }
  
  // success
  free(indxc);
  free(indxr);
  free(ipiv);
  return true;
}
