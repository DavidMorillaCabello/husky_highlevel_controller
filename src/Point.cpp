#include <husky_highlevel_controller/Point.hpp>

namespace husky_highlevel_controller
{

Point::Point()
{
    x_ = 0;
    y_ = 0;
    rho_ = 0;
    th_ = 0;
}

Point Point::fromCartesian(float x, float y)
{
    Point point;
    point.setX(x);
    point.setY(y);
};

Point Point::fromPolar(float rho, float th)
{
    Point point;
    point.setRho(rho);
    point.setTh(th);
};

Point::~Point()
{
}

void Point::setX(double x)
{
    x_ = x;
    recomputePolars_();
}

void Point::setY(double y)
{
    y_ = y;
    recomputePolars_();
}

void Point::setRho(double rho)
{
    rho_ = rho;
    recomputeCartesians_();
}

void Point::setTh(double th)
{
    th_ = th;
    recomputeCartesians_();
}

float Point::getX()
{
    return x_;
}

float Point::getY()
{
    return y_;
}

float Point::getRho()
{
    return rho_;
}

float Point::getTh()
{
    return th_;
}

void Point::recomputePolars_()
{
    rho_ = sqrt(pow(x_, 2) + pow(y_, 2));
    th_ = atan2(y_, x_);
}

void Point::recomputeCartesians_()
{
    x_ = rho_ * cos(th_);
    y_ = rho_ * sin(th_);
}

} // namespace husky_highlevel_controller