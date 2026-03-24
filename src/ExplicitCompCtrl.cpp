#include "ExplicitCompCtrl.h"
#include <SpaceVecAlg/EigenTypedef.h>
#include <mc_rbdyn/Device.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/unique_ptr.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <array>
#include <memory>

ExplicitCompCtrl::ExplicitCompCtrl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;
  tool_frame = "tool";
  if(config.has(robot().module().name))
  {
    auto robot_config = config(robot().module().name);
    robot_config("tool_frame", tool_frame);
  }
  mc_rtc::log::info("Using tool frame: {}", tool_frame);

  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 100});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(
      robots(), robot().robotIndex(), solver().dt(), std::array<double, 3>({0.1, 0.01, 0.5}),
      std::array<double, 2>({1.8, 100}), 0.9, false, true));
  solver().addConstraintSet(dynamicsConstraint);

  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                 {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};

  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("RequestedState", requestedState_);
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr {
    if(postureTask) { return postureTask; }
    return getPostureTask(robot().name());
  });

  addGui();

  logger().addLogEntries(
      this, "tau_ext", [this]() -> Eigen::VectorXd { return this->robot().externalTorques(); }, "tau_comp",
      [this]()
      {
        return this->robot().compensationTorques() ? this->robot().compensationTorques().value()
                                                   : Eigen::VectorXd::Zero(this->robot().mb().nrDof());
      },
      "alphaDext", [this]() { return this->robot().externalTorquesAcc(); }, "alphaDcomp",
      [this]()
      {
        return this->robot().compensationTorquesAcc() ? this->robot().compensationTorquesAcc().value()
                                                      : Eigen::VectorXd::Zero(this->robot().mb().nrDof());
      });

  mc_rtc::log::success("[ExplicitCompCtrl] Init done ");
}

bool ExplicitCompCtrl::run()
{
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
}

void ExplicitCompCtrl::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  requestState("Initial");
}

void ExplicitCompCtrl::switchToInitialState()
{
  auto basePostureTask = getPostureTask(robot().name());

  solver().removeTask(eeTask);
  eeTask.reset();

  solver().removeTask(postureTask);
  postureTask.reset();

  solver().removeTask(basePostureTask);
  if(basePostureTask)
  {
    basePostureTask->reset();
    basePostureTask->setGains(1.0, 2.0);
    basePostureTask->target(postureHome);
    solver().addTask(basePostureTask);
  }

  datastore().assign<std::string>("ControlMode", "Position");
  requestState("Initial");
}

void ExplicitCompCtrl::switchToCompliantState()
{
  auto basePostureTask = getPostureTask(robot().name());
  solver().removeTask(basePostureTask);

  solver().removeTask(postureTask);
  postureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1.0, 1.0);
  postureTask->reset();
  postureTask->setGains(1.0, 2.0);
  postureTask->target(postureHome);
  applyPostureCompliance();
  solver().addTask(postureTask);

  solver().removeTask(eeTask);
  eeTask = std::make_shared<mc_tasks::CompliantEndEffectorTask>(tool_frame, robots(), robot().robotIndex(), 100.0, 1e4);
  eeTask->positionTask->setGains(100.0, 20.0);
  eeTask->orientationTask->setGains(100.0, 20.0);
  eeTask->orientationTask->orientation(Eigen::Quaterniond(1, -1, -1, -1).normalized().toRotationMatrix());
  eeTask->positionTask->position(Eigen::Vector3d(0.6, 0.0, 0.4));
  applyEndEffectorCompliance();
  solver().addTask(eeTask);

  datastore().assign<std::string>("ControlMode", "Torque");
  requestState("Compliant");
}

void ExplicitCompCtrl::requestState(const std::string & state)
{
  requestedState_ = state;
  datastore().assign<std::string>("RequestedState", requestedState_);
}

const std::string & ExplicitCompCtrl::requestedState() const
{
  return requestedState_;
}

void ExplicitCompCtrl::postureCompliance(double gamma)
{
  postureCompliance_ = gamma;
  applyPostureCompliance();
}

double ExplicitCompCtrl::postureCompliance() const
{
  return postureCompliance_;
}

void ExplicitCompCtrl::endEffectorCompliance(double gamma)
{
  endEffectorCompliance_ = gamma;
  applyEndEffectorCompliance();
}

double ExplicitCompCtrl::endEffectorCompliance() const
{
  return endEffectorCompliance_;
}

void ExplicitCompCtrl::addGui()
{
  gui()->removeCategory({"Controller", "ExplicitCompliance"});
  gui()->addElement(
      {"Controller", "ExplicitCompliance"}, mc_rtc::gui::Label("Current mode", [this]() {
        return datastore().get<std::string>("ControlMode");
      }),
      mc_rtc::gui::Button("Switch to initial state", [this]() { requestState("Initial"); }),
      mc_rtc::gui::Button("Switch to compliant state", [this]() { requestState("Compliant"); }));

  gui()->addElement(
      {"Controller", "ExplicitCompliance", "Posture"},
      mc_rtc::gui::NumberInput("Compliance scalar", [this]() { return postureCompliance(); },
                               [this](double gamma) { postureCompliance(gamma); }),
      mc_rtc::gui::NumberSlider("Compliance scalar slider", [this]() { return postureCompliance(); },
                                [this](double gamma) { postureCompliance(gamma); }, 0.0, 1.2));

  gui()->addElement(
      {"Controller", "ExplicitCompliance", "EndEffector"},
      mc_rtc::gui::NumberInput("Compliance scalar", [this]() { return endEffectorCompliance(); },
                               [this](double gamma) { endEffectorCompliance(gamma); }),
      mc_rtc::gui::NumberSlider("Compliance scalar slider", [this]() { return endEffectorCompliance(); },
                                [this](double gamma) { endEffectorCompliance(gamma); }, 0.0, 1.2));
}

void ExplicitCompCtrl::applyPostureCompliance()
{
  if(postureTask)
  {
    postureTask->makeCompliant(Eigen::VectorXd::Constant(robot().mb().nrDof(), postureCompliance_));
  }
}

void ExplicitCompCtrl::applyEndEffectorCompliance()
{
  if(eeTask) { eeTask->setComplianceVector(Eigen::Vector6d::Constant(endEffectorCompliance_)); }
}
