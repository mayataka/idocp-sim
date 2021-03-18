#include "idocp-sim/raisim_wrapper.hpp"

#include "pinocchio/algorithm/frames.hpp"

#include <cassert>

namespace idocp {
namespace sim {

void pin2rai(const pinocchio::Model& pin_model, pinocchio::Data& pin_data, 
             const Eigen::VectorXd& q_pin, const Eigen::VectorXd& v_pin, 
             Eigen::VectorXd& q_rai, Eigen::VectorXd& v_rai) {
  assert(q_pin.size() == 19);
  assert(v_pin.size() == 18);
  assert(q_rai.size() == 19);
  assert(v_rai.size() == 18);
  q_rai.coeffRef(0)  = q_pin.coeff(0);
  q_rai.coeffRef(1)  = q_pin.coeff(1);
  q_rai.coeffRef(2)  = q_pin.coeff(2);
  q_rai.coeffRef(3)  = q_pin.coeff(6);
  q_rai.coeffRef(4)  = q_pin.coeff(3);
  q_rai.coeffRef(5)  = q_pin.coeff(4);
  q_rai.coeffRef(6)  = q_pin.coeff(5);
  q_rai.coeffRef(7)  = q_pin.coeff(7);
  q_rai.coeffRef(8)  = q_pin.coeff(8);
  q_rai.coeffRef(9)  = q_pin.coeff(9);
  q_rai.coeffRef(10) = q_pin.coeff(13);
  q_rai.coeffRef(11) = q_pin.coeff(14);
  q_rai.coeffRef(12) = q_pin.coeff(15);
  q_rai.coeffRef(13) = q_pin.coeff(10);
  q_rai.coeffRef(14) = q_pin.coeff(11);
  q_rai.coeffRef(15) = q_pin.coeff(12);
  q_rai.coeffRef(16) = q_pin.coeff(16);
  q_rai.coeffRef(17) = q_pin.coeff(17);
  q_rai.coeffRef(18) = q_pin.coeff(18);
  pinocchio::framesForwardKinematics(pin_model, pin_data, q_pin);
  // v_rai.template segment<6>(0)
  //     = pin_data.oMf[2].act(v_pin.template segment<6>(0)).toVector();
  v_rai.template segment<3>(0).noalias()
      = pin_data.oMf[2].rotation() * v_pin.template segment<3>(0);
  v_rai.template segment<3>(3).noalias() 
      = pin_data.oMf[2].rotation() * v_pin.template segment<3>(3);
  v_rai.coeffRef(6)  = v_pin.coeff(6);
  v_rai.coeffRef(7)  = v_pin.coeff(7);
  v_rai.coeffRef(8)  = v_pin.coeff(8);
  v_rai.coeffRef(9)  = v_pin.coeff(12);
  v_rai.coeffRef(10) = v_pin.coeff(13);
  v_rai.coeffRef(11) = v_pin.coeff(14);
  v_rai.coeffRef(12) = v_pin.coeff(9);
  v_rai.coeffRef(13) = v_pin.coeff(10);
  v_rai.coeffRef(14) = v_pin.coeff(11);
  v_rai.coeffRef(15) = v_pin.coeff(15);
  v_rai.coeffRef(16) = v_pin.coeff(16);
  v_rai.coeffRef(17) = v_pin.coeff(17);
}


void pin2rai(const Eigen::VectorXd& u_pin, Eigen::VectorXd& u_rai) {
  assert(u_pin.size() == 12);
  assert(u_rai.size() == 18);
  u_rai.template head<6>().setZero();
  u_rai.coeffRef(6)  = u_pin.coeff(0);
  u_rai.coeffRef(7)  = u_pin.coeff(1);
  u_rai.coeffRef(8)  = u_pin.coeff(2);
  u_rai.coeffRef(9)  = u_pin.coeff(6);
  u_rai.coeffRef(10) = u_pin.coeff(7);
  u_rai.coeffRef(11) = u_pin.coeff(8);
  u_rai.coeffRef(12) = u_pin.coeff(3);
  u_rai.coeffRef(13) = u_pin.coeff(4);
  u_rai.coeffRef(14) = u_pin.coeff(5);
  u_rai.coeffRef(15) = u_pin.coeff(9);
  u_rai.coeffRef(16) = u_pin.coeff(10);
  u_rai.coeffRef(17) = u_pin.coeff(11);
}


void rai2pin(const pinocchio::Model& pin_model, pinocchio::Data& pin_data, 
             const Eigen::VectorXd& q_rai, const Eigen::VectorXd& v_rai, 
             Eigen::VectorXd& q_pin, Eigen::VectorXd& v_pin) {
  assert(q_ra.size() == 19);
  assert(v_ra.size() == 18);
  assert(q_pin.size() == 19);
  assert(v_pin.size() == 18);
  q_pin.coeffRef(0)  = q_rai.coeff(0);
  q_pin.coeffRef(1)  = q_rai.coeff(1);
  q_pin.coeffRef(2)  = q_rai.coeff(2);
  q_pin.coeffRef(6)  = q_rai.coeff(3);
  q_pin.coeffRef(3)  = q_rai.coeff(4);
  q_pin.coeffRef(4)  = q_rai.coeff(5);
  q_pin.coeffRef(5)  = q_rai.coeff(6);
  q_pin.coeffRef(7)  = q_rai.coeff(7);
  q_pin.coeffRef(8)  = q_rai.coeff(8);
  q_pin.coeffRef(9)  = q_rai.coeff(9);
  q_pin.coeffRef(13) = q_rai.coeff(10);
  q_pin.coeffRef(14) = q_rai.coeff(11);
  q_pin.coeffRef(15) = q_rai.coeff(12);
  q_pin.coeffRef(10) = q_rai.coeff(13);
  q_pin.coeffRef(11) = q_rai.coeff(14);
  q_pin.coeffRef(12) = q_rai.coeff(15);
  q_pin.coeffRef(16) = q_rai.coeff(16);
  q_pin.coeffRef(17) = q_rai.coeff(17);
  q_pin.coeffRef(18) = q_rai.coeff(18);
  pinocchio::framesForwardKinematics(pin_model, pin_data, q_pin);
  // v_pin.template segment<6>(0)
  //     = pin_data.oMf[2].actInv(v_rai.template segment<6>(0)).toVector();
  v_pin.template segment<3>(0).noalias()
      = pin_data.oMf[2].rotation().transpose() * v_rai.template segment<3>(0);
  v_pin.template segment<3>(3).noalias() 
      = pin_data.oMf[2].rotation().transpose() * v_rai.template segment<3>(3);
  v_pin.coeffRef(6)  = v_rai.coeff(6);
  v_pin.coeffRef(7)  = v_rai.coeff(7);
  v_pin.coeffRef(8)  = v_rai.coeff(8);
  v_pin.coeffRef(9)  = v_rai.coeff(12);
  v_pin.coeffRef(10) = v_rai.coeff(13);
  v_pin.coeffRef(11) = v_rai.coeff(14);
  v_pin.coeffRef(12) = v_rai.coeff(9);
  v_pin.coeffRef(13) = v_rai.coeff(10);
  v_pin.coeffRef(14) = v_rai.coeff(11);
  v_pin.coeffRef(15) = v_rai.coeff(15);
  v_pin.coeffRef(16) = v_rai.coeff(16);
  v_pin.coeffRef(17) = v_rai.coeff(17);
}

} // namespace sim
} // namespace idocp