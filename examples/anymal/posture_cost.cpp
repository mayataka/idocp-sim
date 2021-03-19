#include "posture_cost.hpp"

#include <iostream>
#include <stdexcept>


namespace idocp {
namespace sim {

PostureCost::PostureCost(const Robot& robot) 
  : dimq_(robot.dimq()),
    dimv_(robot.dimv()),
    t_seq_(),
    q_standing_(Eigen::VectorXd::Zero(robot.dimq())),
    q_forward_(Eigen::VectorXd::Zero(robot.dimq())),
    q_backward_(Eigen::VectorXd::Zero(robot.dimq())),
    q_left_(Eigen::VectorXd::Zero(robot.dimq())),
    q_right_(Eigen::VectorXd::Zero(robot.dimq())),
    q_weight_(Eigen::VectorXd::Zero(robot.dimv())),
    qf_weight_(Eigen::VectorXd::Zero(robot.dimv())) {
}


PostureCost::PostureCost()
  : dimq_(0),
    dimv_(0),
    t_seq_(0),
    q_standing_(),
    q_forward_(),
    q_backward_(),
    q_left_(),
    q_right_(),
    q_weight_(),
    qf_weight_() {
}


PostureCost::~PostureCost() {
}


bool PostureCost::useKinematics() const {
  return false;
}


void PostureCost::set_ref_standing(const Eigen::VectorXd& q_ref) {
  q_standing_ = q_ref;
}


void PostureCost::set_ref_forward(const Eigen::VectorXd& q_ref) {
  q_forward_ = q_ref;
}


void PostureCost::set_ref_backward(const Eigen::VectorXd& q_ref) {
  q_backward_ = q_ref;
}


void PostureCost::set_ref_left(const Eigen::VectorXd& q_ref) {
  q_left_ = q_ref;
}


void PostureCost::set_ref_right(const Eigen::VectorXd& q_ref) {
  q_right_ = q_ref;
}


void PostureCost::set_switch_time(const std::vector<double>& t_seq) {
  t_seq_ = t_seq;
}


void PostureCost::set_q_weight(const Eigen::VectorXd& q_weight) {
  q_weight_ = q_weight;
}


void PostureCost::set_qf_weight(const Eigen::VectorXd& qf_weight) {
  qf_weight_ = qf_weight;
}


double PostureCost::computeStageCost(
    Robot& robot, CostFunctionData& data, const double t, const double dt, 
    const SplitSolution& s) const {
  double l = 0;
  compute_q_ref(robot, t, data.q_ref);
  robot.subtractConfiguration(s.q, data.q_ref, data.qdiff);
  l += (q_weight_.array()*data.qdiff.array()*data.qdiff.array()).sum();
  return 0.5 * dt * l;
}


double PostureCost::computeTerminalCost(
    Robot& robot, CostFunctionData& data, const double t, 
    const SplitSolution& s) const {
  double l = 0;
  compute_q_ref(robot, t, data.q_ref);
  robot.subtractConfiguration(s.q, data.q_ref, data.qdiff);
  l += (qf_weight_.array()*data.qdiff.array()*data.qdiff.array()).sum();
  return 0.5 * l;
}


double PostureCost::computeImpulseCost(
    Robot& robot, CostFunctionData& data, const double t, 
    const ImpulseSplitSolution& s) const {
  return 0;
}


void PostureCost::computeStageCostDerivatives(
    Robot& robot, CostFunctionData& data, const double t, const double dt, 
    const SplitSolution& s, SplitKKTResidual& kkt_residual) const {
  compute_q_ref(robot, t, data.q_ref);
  robot.subtractConfiguration(s.q, data.q_ref, data.qdiff);
  robot.dSubtractdConfigurationPlus(s.q, data.q_ref, data.J_qdiff);
  kkt_residual.lq().noalias()
      += dt * data.J_qdiff.transpose() * q_weight_.asDiagonal() * data.qdiff;
}


void PostureCost::computeTerminalCostDerivatives(
    Robot& robot, CostFunctionData& data, const double t, 
    const SplitSolution& s, SplitKKTResidual& kkt_residual) const {
  compute_q_ref(robot, t, data.q_ref);
  robot.subtractConfiguration(s.q, data.q_ref, data.qdiff);
  robot.dSubtractdConfigurationPlus(s.q, data.q_ref, data.J_qdiff);
  kkt_residual.lq().noalias()
      += data.J_qdiff.transpose() * qf_weight_.asDiagonal() * data.qdiff;
}


void PostureCost::computeImpulseCostDerivatives(
    Robot& robot, CostFunctionData& data, const double t, 
    const ImpulseSplitSolution& s, 
    ImpulseSplitKKTResidual& kkt_residual) const {
}


void PostureCost::computeStageCostHessian(
    Robot& robot, CostFunctionData& data, const double t, const double dt, 
    const SplitSolution& s, SplitKKTMatrix& kkt_matrix) const {
  compute_q_ref(robot, t, data.q_ref);
  robot.dSubtractdConfigurationPlus(s.q, data.q_ref, data.J_qdiff);
  kkt_matrix.Qqq().noalias()
      += dt * data.J_qdiff.transpose() * q_weight_.asDiagonal() * data.J_qdiff;
}


void PostureCost::computeTerminalCostHessian(
    Robot& robot, CostFunctionData& data, const double t, 
    const SplitSolution& s, SplitKKTMatrix& kkt_matrix) const {
  compute_q_ref(robot, t, data.q_ref);
  robot.dSubtractdConfigurationPlus(s.q, data.q_ref, data.J_qdiff);
  kkt_matrix.Qqq().noalias()
      += data.J_qdiff.transpose() * qf_weight_.asDiagonal() * data.J_qdiff;
}


void PostureCost::computeImpulseCostHessian(
    Robot& robot, CostFunctionData& data, const double t, 
    const ImpulseSplitSolution& s, ImpulseSplitKKTMatrix& kkt_matrix) const {
}

} // namespace sim
} // namespace idocp