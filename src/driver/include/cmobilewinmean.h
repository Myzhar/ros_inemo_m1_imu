#ifndef CMOBILEWINMEAN_H
#define CMOBILEWINMEAN_H

#include <vector>

using namespace std;

/*!
 * \brief The CMobileWinMean class is used to
 * make a mobile window mean of a sequence of values
 * and reject outliers.
 */
class CMobileWinMean
{
public:
    CMobileWinMean( int winSize );

    int getValCount(){return mValCount;} ///< Return the number of values in the mean vector

    double getMean(){return mMean;} ///< Return the updated mean

    /*!
     * \brief addValue
     * Add a value in the vector and verify if it's an outlier
     * \param val value to be added
     * \return mean value
     */
    double addValue( double val );

private:
    int mWinSize; ///< The size of the window (number of values ti evaluate)
    int mValCount; ///< The number of values in the vector
    vector<double> mLastValues; ///< Circular vector of the last values
    int mInsIdx; ///< The index where to insert the next value

    double mValSum; ///< The updated sum of the values in the vector

    double mMean; ///< The mean of the last \ref mWinSize values
};

#endif // CMOBILEWINMEAN_H
