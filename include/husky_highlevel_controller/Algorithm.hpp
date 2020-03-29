#pragma once 

#include <vector>
#include <algorithm>

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

    private:
        
        //! Internal variable to hold the current minimum...NOT USED
        //double minimum_;

        //! Number of ranges taken. NOT USED
        //size_t nRanges_;

    };
    
} // namespace husky_highlevel_controller