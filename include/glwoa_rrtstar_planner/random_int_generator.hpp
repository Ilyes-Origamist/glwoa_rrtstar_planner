/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#ifndef GLWOA_RRTSTAR_PLANNER_RANDOM_INT_GENERATOR_HPP_  // NOLINT
#define GLWOA_RRTSTAR_PLANNER_RANDOM_INT_GENERATOR_HPP_

#include <random>
#include <climits>  // INT_MAX

namespace glwoa_rrtstar_planner {

/**
 * @brief Class for generating a random integer within a specified range.
 */
class RandomIntGenerator {
 private:
  std::random_device rd_;
  std::mt19937 gen;
  int min_value_{0};
  int max_value_{1};

 public:
  /**
   * @brief Default constructor for the RandomIntGenerator object
   */
  RandomIntGenerator();

  /**
   * @brief Constructor for the RandomIntGenerator object with specified range
   * @param min Minimum value for the range
   * @param max Maximum value for the range
   */
  RandomIntGenerator(int min, int max);

  /**
   * @brief Set the range for the random integer generator
   * @param min Minimum value for the range
   * @param max Maximum value for the range
   */
  void setRange(int min, int max);

  /**
   * @brief Generate a random integer within the specified range
   * @return A random integer
   */
  int generateInt();

  std::uniform_int_distribution<int> dist_;
};
}  // namespace glwoa_rrtstar_planner

#endif  // GLWOA_RRTSTAR_PLANNER_RANDOM_INT_GENERATOR_HPP_  // NOLINT
