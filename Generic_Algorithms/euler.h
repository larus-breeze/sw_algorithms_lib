/** *********************************************************************//**
 * @file		euler.h
 * @brief		Euler angle class implementation
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef EULER_H
#define EULER_H

//! eulerangle pseudo class : equals vector <3>
template <class datatype> class eulerangle
{
public:
	eulerangle( datatype ir=0, datatype ip=0, datatype iy=0)
	: roll(ir), pitch(ip), yaw(iy)
	{}
	datatype roll,pitch,yaw;
};

#endif
