#include "idocp-sim/sim_data_logger.hpp"

#include <assert.h>


namespace idocp {
namespace sim {

SimDataLogger::SimDataLogger(const std::string& save_dir_path, 
                             const std::string& save_file_name) 
  : q_file_(save_dir_path+"/"+save_file_name+"_q.dat"),
    v_file_(save_dir_path+"/"+save_file_name+"_v.dat"),
    u_file_(save_dir_path+"/"+save_file_name+"_u.dat"),
    KKT_error_file_(save_dir_path+"/"+save_file_name+"_KKT_error.dat"),
    conditions_file_(save_dir_path+"/"+save_file_name+"conditions.dat") {
}


SimDataLogger::~SimDataLogger() {
  q_file_.close();
  v_file_.close();
  u_file_.close();
  KKT_error_file_.close();
  conditions_file_.close();
}


void SimDataLogger::save(const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
                         const Eigen::VectorXd& u, const double KKT_error) {
  assert(KKT_error >= 0);
  for (int i=0; i<q.size(); ++i) {
    q_file_ << q[i] << " ";
  }
  q_file_ << "\n";
  for (int i=0; i<v.size(); ++i) {
    v_file_ << v[i] << " ";
  }
  v_file_ << "\n";
  for (int i=0; i<u.size(); ++i) {
    u_file_ << u[i] << " ";
  }
  u_file_ << "\n";
  KKT_error_file_ << KKT_error << "\n";
}


void SimDataLogger::saveConditions(
    const double simulation_time_in_sec, 
    const double sampling_period_in_millisec, 
    const double CPU_time_per_update_in_millisec) {
  assert(simulation_time_in_sec > 0);
  assert(sampling_period_in_millisec > 0);
  assert(CPU_time_per_update_in_millisec > 0);
  conditions_file_ << "simulation time: " << simulation_time_in_sec << "[s]\n";
  conditions_file_ << "sampling time: " 
                  << sampling_period_in_millisec << "[ms]\n";
  conditions_file_ << "CPU time per update: " 
                  << CPU_time_per_update_in_millisec << "[ms]\n";
}

} // namespace sim
} // namespace idocp
