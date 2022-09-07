/***********************************************************************//**
 * @file		matrix.h
 * @brief		matrix class implementation
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef MATRIX_H
#define MATRIX_H

#include "vector.h"

template <class datatype, int size> class vector;

//! mathematical square matrix class
template <class datatype, int size> class matrix
   {
public:
//! default constructor creates unity matrix
   matrix(void);
   //! constructor from array data
   //! elements are assumed to come line by line
   matrix( const datatype *data);
      //! constructor from array data
      //! elements are assumed to come line by line
   matrix( const datatype data[size][size]);
//! copy constructor
   matrix( const matrix & right);

//! copy assignment operator
   matrix & operator =  ( const matrix & right);
   //! multiplication (matrix times vector) -> vector
 vector <datatype, size> operator *( const vector <datatype, size> & right) const;
 //! multiplication (matrix times vector) -> vector
 vector <datatype, size> reverse_map( const vector <datatype, size> & right) const;
//! matrix transposition
      matrix<datatype, size> transpose(void);

//#ifdef DEBUG
//! dump to cout debug helper function
   void print(void);
//#endif
//protected:
//! matrix implementation as 2 dimensional array of datatype
   datatype e[size][size];
   };


template <class datatype, int size> matrix <datatype, size>::matrix()
   {
   for( int i=0; i<size; ++i)
      for( int k=0; k<size; ++k)
         e[i][k]=(i==k) ? 1.0 : 0.0;
   }

template<class datatype, int size>
  matrix<datatype, size>::matrix (const datatype data[size][size])
  {
  for (int k = 0; k < size; ++k)
    for (int i = 0; i < size; ++i)
      e[k][i] = data[k][i];
  }

template<class datatype, int size>
  matrix<datatype, size>::matrix (const datatype *data)
  {
    if (data == 0) // create unity matrix if no initialization
      for (int k = 0; k < size; ++k)
        for (int i = 0; i < size; ++i)
  	e[k][i] = i==k ? 1.0 : 0.0;
    else
      for (int k = 0; k < size; ++k)
	for (int i = 0; i < size; ++i)
	  e[k][i] = *data++;
  }
   
// copy constructor
template <class datatype, int size> matrix <datatype, size>::matrix( const matrix <datatype, size> & right)
   {
   for( int k=0; k<size; ++k)
      for( int i=0; i<size; ++i)
         e[i][k]=right.e[i][k];
   }

template <class datatype, int size> matrix <datatype, size> & matrix <datatype, size>::operator =( const matrix <datatype, size> & right)
   {
   for( int i=0; i<size; ++i)
      for( int k=0; k<size; ++k)
         e[i][k]=right.e[i][k];
   return *this;
   }

template <class datatype, int size>
 vector <datatype, size> matrix <datatype, size>::operator *( const vector <datatype, size> & right) const   //returns a vector<datatype, size> and
   {                                                                      //actual object is matrix<datatype, size>
   vector <datatype, size> retv;
   datatype tmp;
   for( int row=0; row<size; ++row)
      {
      tmp=0.0;
      for( int col=0; col<size; ++col)
         tmp+=e[row][col]*right.e[col];
      retv.e[row]=tmp;
      }
   return retv;
   }

template <class datatype, int size>
 vector <datatype, size> matrix <datatype, size>::reverse_map( const vector <datatype, size> & right)  const //returns a vector<datatype, size> and
   {                                                                      //actual object is matrix<datatype, size>
   vector <datatype, size> retv;
   datatype tmp;
   for( int row=0; row<size; ++row)
      {
      tmp=0.0;
      for( int col=0; col<size; ++col)
         tmp+=e[col][row]*right.e[col];
      retv.e[row]=tmp;
      }
   return retv;
   }

#endif
