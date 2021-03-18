#ifndef IDOCP_SIM_IDOCP_SIM_HPP_
#define IDOCP_SIM_IDOCP_SIM_HPP_

#include <memory>

#include "Eigen/Core"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "idocp-sim/sim_data_logger.hpp"
#include "idocp-sim/raisim_wrapper.hpp"


namespace idocp {
namespace sim {

class MPCCallbackBase {
public:
  MPCCallbackBase() {}

  virtual ~MPCCallbackBase() {}

  virtual void updateControlInput(const double t,
                                  const Eigen::VectorXd& q_pin,
                                  const Eigen::VectorXd& v_pin,
                                  Eigen::VectorXd& u_pin) = 0;

  virtual void computeKKTResidual(const double t,
                                  const Eigen::VectorXd& q_pin,
                                  const Eigen::VectorXd& v_pin) = 0;

  virtual double KKTError() = 0;

private:
};


class idocpSim {
public:
  explicit idocpSim(const std::string& path_to_raisim_activation_key,
                    const std::string& path_to_urdf, 
                    const std::string& path_to_log, 
                    const std::string& sim_name);

  ~idocpSim();

  void setCallback(const std::shared_ptr<MPCCallbackBase>& mpc_callback);

  void runSim(const double simulation_time_in_sec, 
              const double sampling_period_in_sec, 
              const double simulation_start_time_in_sec, 
              const Eigen::VectorXd& q_pin_ini, 
              const Eigen::VectorXd& v_pin_ini,
              const bool visualization, const bool recording);

  static void setupRaiVisCallback();

private:
  std::string path_to_raisim_activation_key_, path_to_urdf_, 
              path_to_log_, sim_name_; 
  SimDataLogger logger_;
  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;
  std::shared_ptr<MPCCallbackBase> mpc_callback_;

};

} // namespace sim
} // namespace idocp

#endif // IDOCP_SIM_IDOCP_SIM_HPP_ 