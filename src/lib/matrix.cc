/*:matrix.cc
********************************************************
* Matrix computation subroutines
* 
* This class is used to do matrix-related manipulations
* for GPS navigation algorithm
*
* This file is original developped by
*      Jay Farrell, Tony Givargis, Elmer Thomas, UCR
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Jan, 2005
*******************************************************/
#include <math.h>
#include <iostream>
#include <string.h>

#include "includes/matrix.h"

const int Matrix::MAX_DIM =  33;
/*
 *initialize a identity matrix
 */
void Matrix::matrix_identity( double* o, int r, int c ) {
    for( int i = 0; i < r; i++){
        for( int j = 0; j < c; j++){
            o[i * c + j] = i== j ? 1.0 : 0.0;
        }
    }
}
/*
 *copy one matrix to another matrix
 */
void Matrix::matrix_copy( double* o, const double* m, int r, int c ) {
    memcpy( o, m, r * c * sizeof(double) );
}

/*
 *add one matrix to another one
 */
void Matrix::matrix_add( double* o, const double* m1, const double* m2, int r, int c ) {
    int n = 0;
    int i = 0;
    for( n = r * c, i = 0; i < n; i++){
        o[i] = m1[i] + m2[i];
    }
}
/*
 *substract one matrix from another one
 */
void Matrix::matrix_sub( double* o, const double* m1, const double* m2, int r, int c ) {
    int n = 0;
    int i = 0;
    for( n = r * c, i = 0; i < n; i++ ){
        o[i] = m1[i] - m2[i];
    }
}

/*
 *matrix multiplication
 */
void Matrix::matrix_mul( double* o, const double* m1, const double* m2, int r, int m, int c ) {
    int k = 0;
    for( int i = 0; i < r; i++ ){
        for( int j = 0; j < c; j++){
            for(o[i * c + j] = 0.0, k = 0; k < m; k++){
                o[i * c + j] += m1[i * m + k] * m2[k * c + j];
            }
        }
    }
}

/*
 *matrix multiplied by one scaler
 */
void Matrix::matrix_muls( double* o, const double* m, double s, int r, int c ) {
    int n = 0;
    int i = 0;
    for( n = r * c, i = 0; i < n; i++ )
        o[i] = m[i] * s;
}

/*
 *transpose one matrix
 */
void Matrix::matrix_transpose( double* o, const double* m, int r, int c ) {
    for( int i = 0; i < r; i++ ){
        for( int j = 0; j < c; j++ ){
            o[j * r + i] = m[i * c + j];
        }
    }
}

/*
 *calculate the inverse of one matrix
 */
int Matrix::matrix_inverse( double* o, const double* m, int n ) {
    int i=0, j=0;
    int index[MAX_DIM];
    double col[MAX_DIM];
    double t[MAX_DIM*MAX_DIM];
    for(i=0;i<MAX_DIM;i++){
        index[i]= 0;
        col[i]= 0;
        for(j=0;j<MAX_DIM;j++){
            t[i*MAX_DIM+j]=0;
        }
    }

    if ( n> MAX_DIM ){
        cerr<<" Matrix Dimension is bigger than MAX_DIM!"<<endl;
        return 0;
    }

    for(i=0; i<n; i++){
        for(j=0; j<n; j++)
            t[i*n+j] = m[i*n+j];
    }
    if( lud(t, n, index) ){
        for(j=0; j<n; j++){
            for(i=0; i<n; i++)
                col[i] = 0.0;
            col[j] = 1.0;
            lub(t, n, index, col);
            for(i=0; i<n; i++)
                o[i*n+j] = col[i];
        }
        return 1;
    }
    else
        return 0;
}

int Matrix::lud(double* m, int n, int* index) {
    int i=0, j=0, k=0, i_max = 0;
    double d_max=0, d_temp=0, d_dum=0;
    double pivot[MAX_DIM];
    for(i=0;i<MAX_DIM;i++){
        pivot[i]=0;
    }

    for(i=0; i<n; i++){
        for(d_max = 0.0, j=0; j<n; j++){
            d_temp = fabs(m[i*n+j]);
            if( d_temp > d_max )
		d_max = d_temp;
        }
        if( d_max == 0.0 ){
            return 0;
        }
        pivot[i] = 1.0 / d_max;
    }
   
    for(j=0; j<n; j++){
        for(i=0; i<j; i++){
            for(d_temp = m[i*n+j], k=0; k<i; k++){
		d_temp -= m[i*n+k] * m[k*n+j];
            }
            m[i*n+j] = d_temp;
        }
        for(d_max=0.0, i=j; i<n; i++){
            for(d_temp = m[i*n+j], k=0; k<j; k++){
		d_temp -= m[i*n+k] * m[k*n+j];
            }
            m[i*n+j] = d_temp;
            d_dum = pivot[i] * fabs(d_temp);
            if( d_dum >= d_max ){
		d_max = d_dum;
		i_max = i;
            }
        }
        if( j != i_max ){
            for(k=0; k<n; k++){
		d_dum = m[i_max*n+k];
		m[i_max*n+k] = m[j*n+k];
		m[j*n+k] = d_dum;
            }
            pivot[i_max] = pivot[j];
        }
        index[j] = i_max;
        if( m[j*n+j] == 0.0 ){
            m[j*n+j] = 1e-20;
        }
        if( j != n ){
            d_dum = 1.0 / m[j*n+j];
            for(i=j+1; i<n; i++){
		m[i*n+j] *= d_dum;
            }
        }
    }
     
    return 1;
}

void Matrix::lub( double* m, int n, int* index, double* b ) {
    int ip = 0;
    int k=0, j = 0, i=0;
    double d_sum = 0.0;
    for(  i = 0; i < n; i++ ){
        ip = index[i];
        d_sum = b[ip];
        b[ip] = b[i];	
        if( k ){
            for( int j = k - 1; j < i; j++ ){
                d_sum -= m[i * n + j] * b[j];
            }
        } else if( d_sum ) {
            k = i + 1;
        }
        b[i] = d_sum;
    }
    for( i = n - 1; i >= 0; i-- ){
        for( d_sum = b[i], j = i + 1; j < n; j++ ){
            d_sum -= m[i * n + j] * b[j];
        }
        b[i] = d_sum / m[i * n + i];
    }

}

double Matrix::median_(double *x, int m){
    int i=0,j=0;
    int min=0;
    double temp=0.0;
    for( i=0; i<m; i++){ 
        min=i;
        for( j=i; j<m; j++){
            if( x[min] > x[j] ){
                min=j;
            }
        }
        temp=x[i];
        x[i]=x[min];
        x[min]=temp;
    }
    i=(m-1)/2;
    j=m/2;
   
    return (x[i]+x[j])/2.0;
}
