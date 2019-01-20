#ifndef MATRIX_H
#define MATRIX_H

using namespace std;

class Matrix{
 private:
  static const int MAX_DIM; // =  33; /**< Maximum dimension for a matrix inversion */
  static const int PIVOT_INDEX;// = sizeof(double);
  void lub(double* , int , int* , double* );
  int  lud(double* , int , int* );
 public:
  /**
   * \param r = number of row elements
   * \param c = number of column elements
   * \param o = identity(m) 
   */
  void matrix_identity( double* , int , int );
  /**
   * \param m = source matrix
   * \param r = number of row elements
   * \param c = number of column elements
   * \param o = destination matrix
   */
  void matrix_copy( double* , const double* , int , int );
  /**
   * \param m1 = matrix 1
   * \param m2 = matrix 2
   * \param r = number of row elements
   * \param c = number of column elements
   * \param o = m1 + m2
   */
  void matrix_add( double* , const double* , const double* , int , int );
  /**
   * \param m1 = matrix 1
   * \param m2 = matrix 2
   * \param r = number of row elements
   * \param c = number of column elements
   * \param o = m1 - m2
   */
  void matrix_sub( double* , const double* , const double* , int , int );
  /**
   * \param m1 = matrix 1
   * \param m2 = matrix 2
   * \param r = number of row elements
   * \param m = Number of col elemtens of m1, and row elements of m2.
   * \param c = number of column elements
   * \param o = m1 * m2
   */
  void matrix_mul( double* ,const double* ,const double* ,int ,int ,int );
  /**
   * \param m1 = matrix 1
   * \param s = scalar
   * \param r = number of row elements
   * \param c = number of column elements
   * \param o = m1 * s
   */
  void matrix_muls( double* , const double* , double , int , int );
  /**
   * \param m = matrix 1
   * \param r = number of row elements
   * \param c = number of column elements
   * \param o = transpose of m
   */
  void matrix_transpose( double* , const double* , int , int );
  /**
   * \param m = matrix 1
   * \param n = number of row/column elements
   * \param o = inverse of m
   */
  int matrix_inverse( double* , const double* , int );

  double median_(double *, int);
};

#endif //MATRIX_H
