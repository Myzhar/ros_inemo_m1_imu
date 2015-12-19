#include "cmobilewinmean.h"

#include <math.h>

CMobileWinMean::CMobileWinMean(int winSize)
{
    mLastValues.resize(winSize );
    mValCount = 0;
    mValSum = 0.0;
    mInsIdx = 0;

    mWinSize = winSize;
}

double CMobileWinMean::addValue( double val )
{
    if( mValCount<mWinSize )
    {
        mValSum += val;
        mValCount++;

        mMean = mValSum/mValCount;

        mLastValues[mInsIdx] = val;

        mInsIdx = (++mInsIdx)%mWinSize;

        return val;
    }

    int prevIdx = (mInsIdx-1);
    if( prevIdx<0 )
        prevIdx = mWinSize-1;

    mValSum -= mLastValues[mInsIdx]; // Subtract the replaced value by the sum
    mValSum += val; // Add the new value to the sum
    mLastValues[mInsIdx] = val;

    mInsIdx = (++mInsIdx)%mWinSize;

    mMean = mValSum/mWinSize;

    return mMean;
}

