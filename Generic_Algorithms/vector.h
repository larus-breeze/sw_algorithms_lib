/** *********************************************************************
 * @file		vector.h
 * @brief		vector class definition
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef vector_H
#define vector_H

#ifndef assert
#define assert(x)
#endif

#include "embedded_math.h"

template <class datatype, int size> class matrix;
template <class datatype> class Cbn;
template <class datatype> class quaternion;

//! mathematical vector of arbitrary type and size
template <class datatype, int size>
class vector
{
	friend class matrix<datatype, size>;
	friend class quaternion<datatype>;
	friend class Cbn<datatype>;
public:

	vector( void)
	{
	    for( int i=0; i < size; ++i)
	      e[i]=ZERO;
	}
	vector( const datatype *data);
	vector( const vector & right);

	datatype scalar_multiply( const vector & right) const //!< scalar product
	{
	    datatype retval = 0;
	    for( int i=0; i < size; ++i)
	      retval += right.e[i] * e[i];
	    return retval;
	}

	datatype operator * ( const vector & right) const //!< scalar (dot) product -> scalar
	{
	  return scalar_multiply( right);
	}

	vector vector_multiply( const vector & right) const //!< vector cross product -> vector
	{
	    assert( size == 3); // operation not defined for vectors of other size
	    vector <datatype, size> tmp;
	    tmp.e[0]=e[1]*right.e[2]-e[2]*right.e[1];
	    tmp.e[1]=e[2]*right.e[0]-e[0]*right.e[2];
	    tmp.e[2]=e[0]*right.e[1]-e[1]*right.e[0];
	    return tmp;
	}

	//! vector abs operator returns absolute value
	datatype abs( void) const
	{
		datatype squaresum=datatype();
		for( int i=0; i< size; ++i)
			squaresum += (e[i]*e[i]);
		return (datatype) SQRT( squaresum);
	};

	vector & operator =  ( const vector & right);
	//! set all elements to zero
	void zero( void)
	{
		for( int i=0; i<size; ++i)
			e[i]=0.0;
	}
	vector & negate( void)
	{
		for( int i=0; i<size; ++i)
			e[i] = -e[i];
		return *this;
	}
	vector & operator += ( const vector & right);
	vector & operator -= ( const vector & right);

	vector operator + ( const vector & right) const;
	vector operator - ( const vector & right) const;
	vector operator * ( const datatype & right) const //!< multiply vector by scalar
	{
		vector result = *this;
		for( int i=0; i < size; ++i)
			result.e[i] *= right;
		return result;
	}
	vector & operator *= ( const datatype & right) //!< scale vector by scalar
	{
		*this = *this * right;
		return *this;
	}
	//! subscription operator
	//! index range checked
	//! through assert
	datatype & operator []( const int index)
	{
		assert( index < size);
		return e[index];
	};

	//! vector normalization
	void normalize( void)
	{
		datatype norm = abs();
		norm = 1.0 / norm;
		*this *= norm;
	}

	// protected:
	//! c-style vector[] of "size" elements
	datatype e[size];
};

//! copy constructor
template <class datatype, int size> vector <datatype, size>::vector( const vector <datatype, size> & right)
{
	for( int i=0; i<size; ++i)
		e[i]=right.e[i];
}

//! constructor from datatype []
template <class datatype, int size> vector <datatype, size>::vector( const datatype * init)
{
	if( init==0)
		for( int i=0; i<size; ++i)
			e[i]=datatype();
	else
		for( int i=0; i<size; ++i)
			e[i]=*init++;
}

//! operator +=
template <class datatype, int size> vector <datatype, size> & vector <datatype, size>::operator +=( const vector <datatype, size> & right)
{
	for( int i=0; i<size; ++i)
		e[i]+=right.e[i];
	return *this;
}

//! operator -=
template <class datatype, int size> vector <datatype, size> & vector <datatype, size>::operator -=( const vector <datatype, size> & right)
{
	for( int i=0; i<size; ++i)
		e[i]-=right.e[i];
	return *this;
}

//! operator +
template <class datatype, int size> vector <datatype, size> vector <datatype, size>::operator +( const vector <datatype, size> & right) const
{
	vector <datatype, size> tmp( *this);
	tmp+=right;
	return tmp;
}

#if 0

//! operator times vector cross product returning vector
template <class datatype, int size> vector <datatype, size> vector <datatype, size>::operator *( const vector <datatype, size> & right) const
{
	//	ASSERT( size == 3); // operation not defined for vectors of other size
	vector <datatype, size> tmp;
	tmp.e[0]=e[1]*right.e[2]-e[2]*right.e[1];
	tmp.e[1]=e[2]*right.e[0]-e[0]*right.e[2];
	tmp.e[2]=e[0]*right.e[1]-e[1]*right.e[0];
	return tmp;
}


//! operator * (vector times scalar returning vector)
template <class datatype, int size> vector <datatype, size> vector <datatype, size>::operator *( const double & right) const
{
	vector <datatype, size> tmp( *this);
	tmp*=right;
	return tmp;
}

//! operator *= (vector multiplied by scalar)
template <class datatype, int size> vector <datatype, size> vector <datatype, size>::operator *=( const double & right)
{
	for( int i=0; i<size; ++i)
		e[i]*=right;
	return *this;
}

//! operator /= (vector divided by scalar returning vector)
template <class datatype, int size> vector <datatype, size> vector <datatype, size>::operator /=( const double & right)
{
	for( int i=0; i<size; ++i)
		e[i]/=right;
	return *this;
}
#endif

//! operator - (vector - vector returns vector)
template <class datatype, int size> vector <datatype, size> vector <datatype, size>::operator -( const vector <datatype, size> & right) const
{
	vector <datatype, size> tmp( *this);
	tmp-=right;
	return tmp;
}

//! vector copy operator - size checked through template mechanism 
template <class datatype, int size>
vector <datatype, size> & vector <datatype, size>::operator =( const vector <datatype, size> & right)
{
	for( int i=0; i<size; ++i)
		e[i]=right.e[i];
	return *this;
}

#endif
