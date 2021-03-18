#ifndef IDOCP_SIM_SIM_DATA_LOGGER_HPP_
#define IDOCP_SIM_SIM_DATA_LOGGER_HPP_

#include <string>
#include <fstream>

#include "Eigen/Core"

#include "idocp/robot/robot.hpp"

namespace idocp {
namespace sim {

class SimDataLogger {
public:
  SimDataLogger(const std::string& save_dir_path, 
                const std::string& save_file_name);

  ~SimDataLogger();

  void save(const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
            const Eigen::VectorXd& u, const double KKT_error);

  void saveConditions(const double simulation_time_in_sec, 
                      const double sampling_period_in_millisec, 
                      const double CPU_time_per_update_in_millisec);

private:
  std::ofstream q_file_, v_file_, u_file_, KKT_error_file_, 
                conditions_file_;

};

} // namespace sim
} // namespace idocp

#endif // IDOCP_SIM_SIM_DATA_LOGGER_HPP_ 