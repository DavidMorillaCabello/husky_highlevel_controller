#pragma once

#include <vector>
#include <algorithm>
#include <math.h>

namespace husky_highlevel_controller
{
/*!
     * Class containing the algorithmic part of the package.
     */
class Algorithm
{

public:

    /*!
     * Constructor.
     */
    Algorithm();

    /*!
     * Destructor;
     */
    virtual ~Algorithm();
    
    /*!
     * Add new range data.
     * @param range the new data.
     */
    //void addRange(const double range); NOT USED

    /*!
     * Get the computed minimum of the ranges.
     * @return the minimum of the ranges.
     */
    double getMinimum(const std::vector<float> &ranges);

    /*!
     * Get the index of the minimum of the ranges.
     * @return the index of the minimum of the ranges.
     */
    int getMinimumIndex(const std::vector<float> &ranges);

    /*!
     * Get the computed minimum of the ranges.
     * @param rho the distance to the point.
     * @param alpha the angle from the point.
     * @param x the returned x coordinate of the point.
     * @param y the returned y coordinate of the point.
     */
    void p2c(float rho, float alpha, float &x, float &y);

private:

    //! Internal variable to hold the current minimum...NOT USED
    //double minimum_;

    //! Number of ranges taken. NOT USED
    //size_t nRanges_;
    
};

} // namespace husky_highlevel_controller