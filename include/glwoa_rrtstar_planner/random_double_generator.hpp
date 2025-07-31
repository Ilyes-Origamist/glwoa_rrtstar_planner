/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#ifndef GLWOA_RRTSTAR_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
#define GLWOA_RRTSTAR_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_

#include <random>
#include <cfloat>  // DBL_MAX

/* DONE:
* - Modified to allow different ranges of x and y for non square maps. 
* - Improved runtime by creating the random engine object once in the constructor. 
*/

namespace glwoa_rrtstar_planner {

/**
 * @brief Class for generating two random double numbers (X,Y) within specified ranges. 
 */
class RandomDoubleGenerator {
 private:
  std::random_device rd_;
  std::mt19937 gen;        // Mersenne Twister engine (keep alive between calls)
  // generate two random numbers
  double min_value_1_{-1.0};
  double max_value_1_{1.0};
  double min_value_2_{-1.0};
  double max_value_2_{1.0};

 public:
  /**
   * @brief Default constructor for the RandomDoubleGenerator object
   */
  RandomDoubleGenerator();

  /**
   * @brief Constructor for the RandomDoubleGenerator object with two specified ranges
   * @param min_1 Minimum value for the first range
   * @param max_1 Maximum value for the first range
   * @param min_2 Minimum value for the second range
   * @param max_2 Maximum value for the second range
   */
  RandomDoubleGenerator(double min_1, double max_1, double min_2, double max_2);

  /**
   * @brief Set the range for the first random number
   * @param min_1 Minimum value for the first range
   * @param max_1 Maximum value for the first range
   */
  void setRangeFirst(double min_1, double max_1);

  /**
   * @brief Set the range for the second random number
   * @param min_2 Minimum value for the second range
   * @param max_2 Maximum value for the second range
   */
  void setRangeSecond(double min_2, double max_2);

  /**
   * @brief Generate a random number within the first specified range
   * @return A random number
   */
  double generateFirst();

  /**
   * @brief Generate a second random number within the second specified range
   * @return A second random number
   */
  double generateSecond();
  // double generateZeroToOne();

 private:
  std::uniform_real_distribution<double> dist_first_;
  std::uniform_real_distribution<double> dist_second_;
};
}  // namespace glwoa_rrtstar_planner

#endif  // GLWOA_RRTSTAR_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
