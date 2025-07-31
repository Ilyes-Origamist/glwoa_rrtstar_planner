/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#include "glwoa_rrtstar_planner/random_int_generator.hpp"

namespace glwoa_rrtstar_planner {

/**
 * @brief Default constructor for the RandomIntGenerator object
 */
RandomIntGenerator::RandomIntGenerator() : gen(rd_()) {}

/**
 * @brief Constructor for the RandomIntGenerator object with specified range
 * @param min Minimum value for the range
 * @param max Maximum value for the range
 */
RandomIntGenerator::RandomIntGenerator(int min, int max) : gen(rd_()), min_value_(min), max_value_(max) {
  dist_ = std::uniform_int_distribution<int>{min_value_, max_value_};
}
void RandomIntGenerator::setRange(int min, int max) {
  min_value_ = min;
  max_value_ = max;
  dist_ = std::uniform_int_distribution<int>{min_value_, max_value_};
}

int RandomIntGenerator::generateInt() {
  return dist_(gen);
}

}  // namespace glwoa_rrtstar_planner
