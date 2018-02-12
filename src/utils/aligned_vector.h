#pragma once

#include <Eigen/StdVector>

namespace cnsee {

template<typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

}
