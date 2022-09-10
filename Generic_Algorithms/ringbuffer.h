/***********************************************************************//**
 * @file		ringbuffer.h
 * @brief		ring buffer helper class (template)
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

**************************************************************************/

#ifndef RINGBUFER_H
#define RINGBUFER_H

template <class datatype, unsigned size> class RingBuffer
   {
public:
    RingBuffer( datatype initial_value = datatype())
        {
        pointer = 0;
		for( unsigned i=0; i<size; ++i)
				values[i] = initial_value;
        }

    datatype getValueAt(unsigned point)
        {
        return values[map(point)];
        }

    datatype getPreviousAt(unsigned position)
        {
        return values[mapback(position)];
        }

    inline datatype operator [] (unsigned position)
    {
        return getValueAt( position);
    }

    void pushValue(datatype value)
        {
        setValueAt(0, value);
        ++pointer;
        if (pointer >= size)
            pointer = 0;
        }

void setAllValues(const datatype &value)
        {
    	for( unsigned i=0; i < size; ++i)
    		setValueAt(i, value);
        }

    unsigned GetSize( void)
        {
        return size;
        }
private:
    unsigned map(unsigned pos)
    {
        return (pos + pointer) % size;
    }
    unsigned mapback(unsigned position)
    {
        int index = pointer - position -1;
        if( index < 0)
            index += size;
        return index;
    }
    void setValueAt(unsigned point, const datatype &value)
    {
        values[map(point)] = value;
    }

    unsigned pointer;
    datatype values[size];
    };

#endif
