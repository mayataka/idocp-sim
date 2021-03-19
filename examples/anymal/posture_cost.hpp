#ifndef IDOCP_SIM_POSTURE_COST_HPP_
#define IDOCP_SIM_POSTURE_COST_HPP_

#include "Eigen/Core"

#include "idocp/robot/robot.hpp"
#include "idocp/cost/cost_function_component_base.hpp"
#include "idocp/cost/cost_function_data.hpp"
#include "idocp/ocp/split_solution.hpp"
#include "idocp/ocp/split_kkt_residual.hpp"
#include "idocp/ocp/split_kkt_matrix.hpp"


namespace idocp {
namespace sim {

class PostureCost final : public CostFunctionComponentBase {
public:
  PostureCost(const Robot& robot);

  PostureCost();

  ~PostureCost();

  // Use defalut copy constructor.
  PostureCost(const PostureCost&) = default;

  // Use defalut copy operator.
  PostureCost& operator=(const PostureCost&) = default;

  // Use defalut move constructor.
  PostureCost(PostureCost&&) noexcept = default;

  // Use defalut move assign operator.
  PostureCost& operator=(PostureCost&&) noexcept = default;

  bool useKinematics() const override;

  void set_ref_standing(const Eigen::VectorXd& q_ref);

  void set_ref_forward(const Eigen::VectorXd& q_ref);

  void set_ref_backward(const Eigen::VectorXd& q_ref);

  void set_ref_left(const Eigen::VectorXd& q_ref);

  void set_ref_right(const Eigen::VectorXd& q_ref);

  void set_switch_time(const std::vector<double>& t_seq);

  void set_q_weight(const Eigen::VectorXd& q_weight);

  void set_qf_weight(const Eigen::VectorXd& qf_weight);

  double computeStageCost(Robot& robot, CostFunctionData& data, const double t, 
                          const double dt, const SplitSolution& s) const;

  double computeTerminalCost(Robot& robot, CostFunctionData& data, 
                             const double t, const SplitSolution& s) const;

  double computeImpulseCost(Robot& robot, CostFunctionData& data, 
                            const double t, 
                            const ImpulseSplitSolution& s) const;

  void computeStageCostDerivatives(Robot& robot, CostFunctionData& data, 
                                   const double t, const double dt, 
                                   const SplitSolution& s, 
                                   SplitKKTResidual& kkt_residual) const;

  void computeTerminalCostDerivatives(Robot& robot, CostFunctionData& data, 
                                      const double t, const SplitSolution& s, 
                                      SplitKKTResidual& kkt_residual) const;

  void computeImpulseCostDerivatives(Robot& robot, CostFunctionData& data, 
                                     const double t, 
                                     const ImpulseSplitSolution& s, 
                                     ImpulseSplitKKTResidual& kkt_residual) const;

  void computeStageCostHessian(Robot& robot, CostFunctionData& data, 
                               const double t, const double dt, 
                               const SplitSolution& s, 
                               SplitKKTMatrix& kkt_matrix) const;

  void computeTerminalCostHessian(Robot& robot, CostFunctionData& data, 
                                  const double t, const SplitSolution& s, 
                                  SplitKKTMatrix& kkt_matrix) const;

  void computeImpulseCostHessian(Robot& robot, CostFunctionData& data, 
                                 const double t, const ImpulseSplitSolution& s, 
                                 ImpulseSplitKKTMatrix& kkt_matrix) const;

  void compute_q_ref(const Robot& robot, const double t, 
                     Eigen::VectorXd& q_ref) const {
    if (t < t_seq_[0]) {
      q_ref = q_standing_;
    }
    else if (t < t_seq_[1]) {
      q_ref = q_forward_;
    }
    else if (t < t_seq_[2]) {
      q_ref = q_standing_;
    }
    else if (t < t_seq_[3]) {
      q_ref = q_backward_;
    }
    else if (t < t_seq_[4]) {
      q_ref = q_standing_;
    }
    else if (t < t_seq_[5]) {
      q_ref = q_left_;
    }
    else if (t < t_seq_[6]) {
      q_ref = q_standing_;
    }
    else if (t < t_seq_[7]) {
      q_ref = q_right_;
    }
    else {
      q_ref = q_standing_;
    }
  }

private:
  int dimq_, dimv_;
  std::vector<double> t_seq_;
  Eigen::VectorXd q_standing_, q_forward_, q_backward_, q_left_, q_right_, q_weight_, qf_weight_;

};

} // namespace sim
} // namespace idocp

#endif // IDOCP_SIM_POSTURE_COST_HPP_ 