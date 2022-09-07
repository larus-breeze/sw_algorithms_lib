/** ***********************************************************************
 * @file		ringbuffer.h
 * @brief		generic ring buffer
 * @author		Dr. Klaus Schaefer
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
