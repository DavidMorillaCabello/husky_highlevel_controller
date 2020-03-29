#include <husky_highlevel_controller/Algorithm.hpp>

namespace husky_highlevel_controller
{
    Algorithm::Algorithm()
    {
    }

    Algorithm::~Algorithm()
    {
    }

    double Algorithm::getMinimum(const std::vector<float> &ranges){
        return *std::min_element(ranges.begin(),ranges.end());
    }
} // namespace husky_highlevel_controller
