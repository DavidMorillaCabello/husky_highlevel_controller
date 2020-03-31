#pragma once

#include<math.h>

namespace husky_highlevel_controller
{
    /*!
     * Class containing the point data structure.
     */
    class Point
    {

    public:
    /*
     * Constructor.
     * Initialize the coordinates to 0.
     */
    Point::Point(){
        x_ = 0;
        y_ = 0;
        rho_ = 0;
        th_ = 0;
    }

    /*
     * Constructor in cartesian coordinates.
     * @param x the x coordinate of the point.
     * @param y the y coordinate of the point.
     */
    static Point Point::fromCartesian(float x, float y);

    /*
     * Constructor in polar coordinates.
     * @param rho the distance to the point.
     * @param alpha the angle from the point.
     */
    static Point Point::fromPolar(float rho, float th);

    /*!
     * Destructor;
     */
    virtual ~Point();

    /*
     * Getters & Setters
     */
    void Point::setX(double x);
    void Point::setY(double y);
    void Point::setRho(double rho);
    void Point::setTh(double th);

    float Point::getX();
    float Point::getY();
    float Point::getRho();
    float Point::getTh();

    private:
        
        //! Internal variable to hold the coordinates
        float x_,y_,rho_,th_;

        //! Method to update polar coordinates when cartesians are changed.
        void recomputePolars_();

        //! Method to update cartesian coordinates when polars are changed.
        void recomputeCartesians_();

    };
    
} // namespace husky_highlevel_controller