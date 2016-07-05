#ifndef FILTERVELOCITY_HPP
#define FILTERVELOCITY_HPP
#include <vector>
class FilterVelocity {
 public:
  FilterVelocity();
  double filter(double new_value);

 private:
  double m_size_filter;
  std::vector<double> m_coeffs;
  std::vector<double> m_input_buffer;
};

#endif  // FILTERVELOCITY_HPP
