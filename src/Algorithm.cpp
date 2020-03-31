#include <husky_highlevel_controller/Algorithm.hpp>

namespace husky_highlevel_controller
{

Algorithm::Algorithm()
{
}

Algorithm::~Algorithm()
{
}

double Algorithm::getMinimum(const std::vector<float> &ranges)
{
    return *std::min_element(ranges.begin(), ranges.end());
}

int Algorithm::getMinimumIndex(const std::vector<float> &ranges)
{
    return std::min_element(ranges.begin(), ranges.end()) - ranges.begin();
}

void Algorithm::p2c(float rho, float alpha, float &x, float &y)
{
    x = rho * cos(alpha);
    y = rho * sin(alpha);
}

} // namespace husky_highlevel_controller
