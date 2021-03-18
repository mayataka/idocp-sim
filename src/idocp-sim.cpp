#include "idocp-sim/idocp-sim.hpp"

#include "pinocchio/parsers/urdf.hpp"

#include "raisim/World.hpp"
#include "raisim/OgreVis.hpp"
#include "raisim/helper.hpp"


namespace idocp {
namespace sim {

idocpSim::idocpSim(const std::string& path_to_raisim_activation_key,
                   const std::string& path_to_urdf, 
                   const std::string& path_to_log, 
                   const std::string& sim_name)
  : path_to_raisim_activation_key_(path_to_raisim_activation_key), 
    path_to_urdf_(path_to_urdf), 
    path_to_log_(path_to_log), 
    sim_name_(sim_name),
    logger_(path_to_log, sim_name),
    pin_model_(),
    pin_data_(),
    mpc_callback_() {
  pinocchio::urdf::buildModel(path_to_urdf, pinocchio::JointModelFreeFlyer(), pin_model_);
  pin_data_ = pinocchio::Data(pin_model_);
}


idocpSim::~idocpSim() {
}


void idocpSim::setCallback(
    const std::shared_ptr<MPCCallbackBase>& mpc_callback) {
  mpc_callback_ = mpc_callback;
}


void idocpSim::runSim(const double simulation_time_in_sec, 
                      const double sampling_period_in_sec, 
                      const double simulation_start_time_in_sec, 
                      const Eigen::VectorXd& q_pin_ini, 
                      const Eigen::VectorXd& v_pin_ini,
                      const bool visualization, const bool recording) {
  try {
    if (simulation_time_in_sec <= 0) {
      throw std::out_of_range(
          "Invalid argument: simulation_time_in_sec must be positive!");
    }
    if (sampling_period_in_sec <= 0) {
      throw std::out_of_range(
          "Invalid argument: sampling_period_in_sec must be positive!");
    }
    if (simulation_start_time_in_sec < 0) {
      throw std::out_of_range(
          "Invalid argument: simulation_start_time_in_sec must be non negative!");
    }
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
  raisim::World::setActivationKey(path_to_raisim_activation_key_);
  raisim::World rai_world;
  auto rai_robot = rai_world.addArticulatedSystem(path_to_urdf_, "");
  auto rai_ground = rai_world.addGround();
  rai_world.setTimeStep(sampling_period_in_sec);
  rai_world.setERP(0.2, 0.2);
  rai_world.setDefaultMaterial(1000, 0, 0);
  auto rai_vis = raisim::OgreVis::get();
  if (visualization) {
    rai_vis->setWorld(&rai_world);
    rai_vis->setSetUpCallback(setupRaiVisCallback);
    rai_vis->setAntiAliasing(2);
    rai_vis->setWindowSize(500, 400);
    rai_vis->setContactVisObjectSize(0.025, 0.01); 
    rai_vis->initApp();
    rai_vis->createGraphicalObject(rai_ground, 20, "floor", "checkerboard_green");
    rai_vis->createGraphicalObject(rai_robot, "ANYmal");
    const double scaled_position = 0.6;
    rai_vis->getCameraMan()->getCamera()->setPosition(-3.5*scaled_position, 
                                                      -3.5*scaled_position, 
                                                      2*scaled_position);
    rai_vis->getCameraMan()->getCamera()->rotate(Ogre::Vector3(1.0, 0, 0), 
                                                 Ogre::Radian(M_PI_2));
    rai_vis->getCameraMan()->getCamera()->rotate(Ogre::Vector3(-1.0, -1.0, 0.), 
                                                 Ogre::Radian(0.3));
    rai_vis->getCameraMan()->getCamera()->rotate(Ogre::Vector3(0, -1.0, 0), 
                                                 Ogre::Radian(0.5));
    rai_vis->getCameraMan()->getCamera()->rotate(Ogre::Vector3(0, 0, -1.0), 
                                                 Ogre::Radian(0.2));
  }
  Eigen::VectorXd q_pin = q_pin_ini;
  Eigen::VectorXd v_pin = v_pin_ini;
  Eigen::VectorXd u_pin = Eigen::VectorXd::Zero(v_pin.size()-6);
  Eigen::VectorXd q_rai = Eigen::VectorXd::Zero(q_pin.size());
  Eigen::VectorXd v_rai = Eigen::VectorXd::Zero(v_pin.size());
  Eigen::VectorXd u_rai = Eigen::VectorXd::Zero(v_pin.size());
  mpc_callback_->updateControlInput(0, q_pin, v_pin, u_pin);
  pin2rai(pin_model_, pin_data_, q_pin, v_pin, q_rai, v_rai);
  pin2rai(u_pin, u_rai);
  rai_robot->setState(q_rai, v_rai);
  rai_robot->setGeneralizedForce(u_rai);
  if (visualization && recording) {
    rai_vis->startRecordingVideo(path_to_log_+"/"+sim_name_+".mp4");
  }
  std::chrono::system_clock::time_point start_clock, end_clock;
  double CPU_time_total_in_sec = 0;
  for (double t=0; t<simulation_time_in_sec; t+=sampling_period_in_sec) {
    rai_robot->setGeneralizedForce(u_rai);
    rai_world.integrate();
    if (visualization) {
      rai_vis->renderOneFrame();
    }
    rai_robot->getState(q_rai, v_rai);
    rai2pin(pin_model_, pin_data_, q_rai, v_rai, q_pin, v_pin);
    mpc_callback_->computeKKTResidual(t, q_pin, v_pin);
    logger_.save(q_pin, v_pin, u_pin, mpc_callback_->KKTError());
    start_clock = std::chrono::system_clock::now();
    mpc_callback_->updateControlInput(t, q_pin, v_pin, u_pin);
    pin2rai(u_pin, u_rai);
    end_clock = std::chrono::system_clock::now();
    const double CPU_time_in_sec
        = 1.0e-06 * std::chrono::duration_cast<std::chrono::microseconds>(
            end_clock-start_clock).count();
    CPU_time_total_in_sec += CPU_time_in_sec;
  }
  const int num_simulation_update 
      = (int)(simulation_time_in_sec/sampling_period_in_sec);
  const double CPU_time_per_update_in_sec 
      = CPU_time_total_in_sec / num_simulation_update;
  logger_.saveConditions(simulation_time_in_sec, 1000*sampling_period_in_sec,
                         1000*CPU_time_per_update_in_sec);
  std::cout << "simulation time: " << simulation_time_in_sec << "[s]" 
            << std::endl;
  std::cout << "sampling time: " << 1000 * sampling_period_in_sec << "[ms]" 
            << std::endl;
  std::cout << "CPU time per update: " << 1000 * CPU_time_per_update_in_sec 
            << "[ms]" << std::endl;
  if (visualization && recording) {
    rai_vis->stopRecordingVideoAndSave();
  }
  rai_vis->closeApp();
}


void idocpSim::setupRaiVisCallback() {
  auto rai_vis = raisim::OgreVis::get();
  /// light
  rai_vis->getLight()->setDiffuseColour(1, 1, 1);
  rai_vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3,3,-0.5);
  lightdir.normalise();
  rai_vis->getLightNode()->setDirection({lightdir});
  rai_vis->setCameraSpeed(300);
  /// load  textures
  rai_vis->addResourceDirectory(rai_vis->getResourceDir() + "/material/checkerboard");
  rai_vis->loadMaterialFile("checkerboard.material");
  /// shdow setting
  rai_vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  rai_vis->getSceneManager()->setShadowTextureSettings(2048, 3);
  /// scale related settings!! Please adapt it depending on your map size
  /// beyond this distance, shadow disappears
  rai_vis->getSceneManager()->setShadowFarDistance(10);
  // size of contact points and contact forces
  rai_vis->setContactVisObjectSize(0.03, 0.6);
  // speed of camera motion in freelook mode
  rai_vis->getCameraMan()->setTopSpeed(5);
  /// skybox
  Ogre::Quaternion quat;
  quat.FromAngleAxis(Ogre::Radian(0.5*M_PI_2), {1., 0, 0});
  rai_vis->getSceneManager()->setSkyBox(true, "Examples/StormySkyBox", 500, true, quat);
}

} // namespace sim
} // namespace idocp