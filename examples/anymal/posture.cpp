#include <iostream>
#include <string>
#include <memory>

#include "Eigen/Core"

#include "idocp/robot/robot.hpp"
#include "idocp/ocp/ocp_solver.hpp"
#include "idocp/cost/cost_function.hpp"
#include "idocp/cost/configuration_space_cost.hpp"
#include "idocp/constraints/constraints.hpp"
#include "idocp/constraints/joint_position_lower_limit.hpp"
#include "idocp/constraints/joint_position_upper_limit.hpp"
#include "idocp/constraints/joint_velocity_lower_limit.hpp"
#include "idocp/constraints/joint_velocity_upper_limit.hpp"
#include "idocp/constraints/joint_torques_lower_limit.hpp"
#include "idocp/constraints/joint_torques_upper_limit.hpp"
#include "idocp/constraints/linearized_friction_cone.hpp"
#include "idocp/constraints/linearized_impulse_friction_cone.hpp"

#include "idocp-sim/idocp-sim.hpp"


class MPCCallback : public idocp::sim::MPCCallbackBase {
public:
  MPCCallback(const idocp::Robot& robot, const idocp::OCPSolver& ocp_solver)
    : robot_(robot),
      contact_status_(robot.createContactStatus()),
      contact_points_(robot.maxPointContacts(), Eigen::Vector3d::Zero()),
      ocp_solver_(ocp_solver) {
    contact_status_.activateContacts({0, 1, 2, 3});
  }

  ~MPCCallback() {}

  void init(const double t, const Eigen::VectorXd& q_pin, 
            const Eigen::VectorXd& v_pin) {
    setContactPointsToOCPSolver(q_pin);
    const int max_iter = 100;
    for (int i=0; i<max_iter; ++i) {
      ocp_solver_.updateSolution(t, q_pin, v_pin);
    }
  }

  void updateControlInput(const double t, const Eigen::VectorXd& q_pin,
                          const Eigen::VectorXd& v_pin,
                          Eigen::VectorXd& u_pin) override {
    setContactPointsToOCPSolver(q_pin);
    ocp_solver_.updateSolution(t, q_pin, v_pin);
    u_pin = ocp_solver_.getSolution(0).u;
  }

  void computeKKTResidual(const double t, const Eigen::VectorXd& q_pin,
                          const Eigen::VectorXd& v_pin) override {
    setContactPointsToOCPSolver(q_pin);
    ocp_solver_.computeKKTResidual(t, q_pin, v_pin);
  }


  double KKTError() override {
    return ocp_solver_.KKTError();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  idocp::Robot robot_;
  idocp::ContactStatus contact_status_;
  std::vector<Eigen::Vector3d> contact_points_;
  idocp::OCPSolver ocp_solver_;

  void setContactPointsToOCPSolver(const Eigen::VectorXd& q_pin) {
    robot_.updateFrameKinematics(q_pin);
    robot_.getContactPoints(contact_points_);
    contact_status_.setContactPoints(contact_points_);
    ocp_solver_.setContactStatusUniformly(contact_status_);
  }

};


int main(int argc, char *argv[]) {
  std::vector<int> contact_frames = {14, 24, 34, 44}; // LF, LH, RF, RH
  const std::string path_to_urdf = "../../../examples/anymal/anymal_b_simple_description/urdf/anymal.urdf";
  idocp::Robot robot(path_to_urdf, contact_frames);

  auto cost = std::make_shared<idocp::CostFunction>();
  Eigen::VectorXd q_standing(Eigen::VectorXd::Zero(robot.dimq()));
  q_standing <<    0,    0, 0.4792, 0, 0, 0, 1, 
                -0.1,  0.7,   -1.0, 
                -0.1, -0.7,    1.0, 
                 0.1,  0.7,   -1.0, 
                 0.1, -0.7,    1.0;
  Eigen::VectorXd q_weight(Eigen::VectorXd::Zero(robot.dimv()));
  q_weight << 1000, 1000, 1000, 1000, 1000, 1000, 
              1, 1, 1,
              1, 1, 1,
              1, 1, 1,
              1, 1, 1;
  Eigen::VectorXd v_weight(Eigen::VectorXd::Zero(robot.dimv()));
  v_weight << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 
              0.1, 0.1, 0.1,
              0.1, 0.1, 0.1,
              0.1, 0.1, 0.1,
              0.1, 0.1, 0.1;
  Eigen::VectorXd a_weight(Eigen::VectorXd::Zero(robot.dimv()));
  a_weight << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 
              0.001, 0.001, 0.001,
              0.001, 0.001, 0.001,
              0.001, 0.001, 0.001,
              0.001, 0.001, 0.001;

  auto config_cost = std::make_shared<idocp::ConfigurationSpaceCost>(robot);
  Eigen::VectorXd q_ref = q_standing;
  config_cost->set_q_weight(q_weight);
  config_cost->set_qf_weight(q_weight);
  config_cost->set_qi_weight(q_weight);
  config_cost->set_v_weight(v_weight);
  config_cost->set_vf_weight(v_weight);
  config_cost->set_vi_weight(v_weight);
  config_cost->set_a_weight(a_weight);
  config_cost->set_dvi_weight(a_weight);
  config_cost->set_q_ref(q_ref);
  cost->push_back(config_cost);

  auto constraints           = std::make_shared<idocp::Constraints>();
  auto joint_position_lower  = std::make_shared<idocp::JointPositionLowerLimit>(robot);
  auto joint_position_upper  = std::make_shared<idocp::JointPositionUpperLimit>(robot);
  auto joint_velocity_lower  = std::make_shared<idocp::JointVelocityLowerLimit>(robot);
  auto joint_velocity_upper  = std::make_shared<idocp::JointVelocityUpperLimit>(robot);
  auto joint_torques_lower   = std::make_shared<idocp::JointTorquesLowerLimit>(robot);
  auto joint_torques_upper   = std::make_shared<idocp::JointTorquesUpperLimit>(robot);
  const double mu = 0.7;
  auto friction_cone         = std::make_shared<idocp::LinearizedFrictionCone>(robot, mu);
  auto impulse_friction_cone = std::make_shared<idocp::LinearizedImpulseFrictionCone>(robot, mu);
  constraints->push_back(joint_position_lower);
  constraints->push_back(joint_position_upper);
  constraints->push_back(joint_velocity_lower);
  constraints->push_back(joint_velocity_upper);
  constraints->push_back(joint_torques_lower);
  constraints->push_back(joint_torques_upper);
  // constraints->push_back(friction_cone);
  // constraints->push_back(impulse_friction_cone);

  const double T = 1; 
  const int N = 20;
  const int max_num_impulse_phase = 1;

  const int nthreads = 4;
  const double t = 0;

  const Eigen::VectorXd q = q_standing;
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(robot.dimv());

  idocp::OCPSolver ocp_solver(robot, cost, constraints, T, N, max_num_impulse_phase, nthreads);

  robot.updateFrameKinematics(q_standing);
  std::vector<Eigen::Vector3d> contact_points(robot.maxPointContacts(), Eigen::Vector3d::Zero());
  robot.getContactPoints(contact_points);
  auto contact_status = robot.createContactStatus();
  contact_status.activateContacts({0, 1, 2, 3});
  contact_status.setContactPoints(contact_points);
  ocp_solver.setContactStatusUniformly(contact_status);

  ocp_solver.setSolution("q", q);
  ocp_solver.setSolution("v", v);
  Eigen::Vector3d f_init;
  f_init << 0, 0, 0.25*robot.totalWeight();
  ocp_solver.setSolution("f", f_init);
  ocp_solver.initConstraints(t);

  if (argc != 3) {
    std::cout << "argment must be: ./posture PAHT_TO_RAISIM_ACTIVATION_KEY PATH_TO_URDF_FOR_RAISIM" << std::endl;
    std::exit(1);
  }

  const std::string path_to_raisim_activation_key = argv[1];
  const std::string path_to_urdf_sim = argv[2];
  const std::string path_to_log = "sim_results";
  const std::string sim_name = "posture";
  idocp::sim::idocpSim sim(path_to_raisim_activation_key, path_to_urdf_sim, path_to_log, sim_name);
  sim.setCallback(std::make_shared<MPCCallback>(robot, ocp_solver));
  const double simulation_time_in_sec = 5;
  const double sampling_period_in_sec = 0.0025;
  const double simulation_start_time_in_sec = 0;
  const bool visualization = true;
  const bool recording = false;
  sim.runSim(simulation_time_in_sec, sampling_period_in_sec, 
             simulation_start_time_in_sec, q, v, visualization, recording);

  return 0;
}