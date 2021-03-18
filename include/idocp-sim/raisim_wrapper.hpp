#ifndef IDOCP_SIM_RAISIM_WRAPPER_HPP_
#define IDOCP_SIM_RAISIM_WRAPPER_HPP_

#include "Eigen/Core"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace idocp {
namespace sim {

void pin2rai(const pinocchio::Model& pin_model, pinocchio::Data& pin_data, 
             const Eigen::VectorXd& q_pin, const Eigen::VectorXd& v_pin, 
             Eigen::VectorXd& q_rai, Eigen::VectorXd& v_rai);

void pin2rai(const Eigen::VectorXd& u_pin, Eigen::VectorXd& u_rai);

void rai2pin(const pinocchio::Model& pin_model, pinocchio::Data& pin_data, 
             const Eigen::VectorXd& q_rai, const Eigen::VectorXd& v_rai, 
             Eigen::VectorXd& q_pin, Eigen::VectorXd& v_pin);

} // namespace sim
} // namespace idocp

#endif // IDOCP_SIM_RAISIM_WRAPPER_HPP_ 