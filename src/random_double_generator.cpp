/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#include "glwoa_rrtstar_planner/random_double_generator.hpp"

namespace glwoa_rrtstar_planner {

// In the constructor, initialize the Mersenne Twister generator with a random seed from the random device

RandomDoubleGenerator::RandomDoubleGenerator() : gen(rd_()) {}

RandomDoubleGenerator::RandomDoubleGenerator(double min_1, double max_1, double min_2, double max_2)
    : gen(rd_()), min_value_1_(min_1), max_value_1_(max_1), min_value_2_(min_2), max_value_2_(max_2) {
  // Pre-create distributions to avoid recreating them each time
  dist_first_ = std::uniform_real_distribution<double>(min_value_1_, std::nextafter(max_value_1_, DBL_MAX));
  dist_second_ = std::uniform_real_distribution<double>(min_value_2_, std::nextafter(max_value_2_, DBL_MAX));
}

void RandomDoubleGenerator::setRangeFirst(double min_1, double max_1) {
  min_value_1_ = min_1;
  max_value_1_ = max_1;
  // Pre-create distributions to avoid recreating them each time
  dist_first_ = std::uniform_real_distribution<double>(min_value_1_, std::nextafter(max_value_1_, DBL_MAX));
}

void RandomDoubleGenerator::setRangeSecond(double min_2, double max_2) {
  min_value_2_ = min_2;
  max_value_2_ = max_2;
  // Pre-create distributions to avoid recreating them each time
  dist_second_ = std::uniform_real_distribution<double>(min_value_2_, std::nextafter(max_value_2_, DBL_MAX));
}

double RandomDoubleGenerator::generateFirst() {
  return dist_first_(gen);
}

double RandomDoubleGenerator::generateSecond() {
  return dist_second_(gen);
}

}  // namespace glwoa_rrtstar_planner
