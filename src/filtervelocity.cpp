#include "ardrone_velocity/filtervelocity.hpp"

FilterVelocity::FilterVelocity()
    : m_size_filter(30.0),
      m_coeffs({0.00498902, 0.00567655, 0.00768429, 0.01092849, 0.01526534,
                0.02049766, 0.02638413, 0.03265082, 0.03900430, 0.04514569,
                0.05078516, 0.05565588, 0.05952694, 0.06221459, 0.06359114,
                0.06359114, 0.06221459, 0.05952694, 0.05565588, 0.05078516,
                0.04514569, 0.03900430, 0.03265082, 0.02638413, 0.02049766,
                0.01526534, 0.01092849, 0.00768429, 0.00567655, 0.00498902}) {
  m_input_buffer = std::vector<double>(m_size_filter, 0.0);
}

double FilterVelocity::filter(double new_value) {
  double result = 0.0;
  int i;
  m_input_buffer[0] = new_value;

  for (i = 0; i < m_size_filter; i++) {
    result += m_input_buffer[i] * m_coeffs[i];
  }

  for (i = m_size_filter - 1; i > 0; i--) {
    m_input_buffer[i] = m_input_buffer[i - 1];
  }
  return result;
}
