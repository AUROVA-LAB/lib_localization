#pragma once

#include <iostream>
#include <chrono> // For benchmarking

#define DEBUG_ALLWAYS(x) do { std::cerr << x << std::endl; } while (0)
#define DEBUG(x)  do { std::cerr << x << std::endl; } while (0)
#define START_TIMER(x) //std::chrono::steady_clock::time_point timer_##x_start = std::chrono::steady_clock::now();
#define END_TIMER(x) /*std::chrono::steady_clock::time_point timer_##x_end = std::chrono::steady_clock::now();\
    std::cerr << #x ": " << std::chrono::duration_cast<std::chrono::microseconds>(timer_##x_end - timer_##x_start).count() << " [us]" << std::endl;*/

namespace static_data_association {

template<int Dim> class AssociationProblem;
template<int Dim> class Hypothesis;


}
