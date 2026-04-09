#include "ExplicitCompCtrl.h"
#include <mc_rbdyn/Device.h>
#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/unique_ptr.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <array>
#include <cmath>
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

  commandedData_.eePosition = {0.6, 0.0, 0.4};
  commandedData_.eeQuaternion = {0.5, -0.5, -0.5, -0.5};
  commandedData_.posture = {0.0, 0.262, 3.14, -2.269, 0.0, 0.96, 1.57};
  commandedData_.eeCompliance = endEffectorComplianceCommand_;
  commandedData_.postureCompliance = postureComplianceCommand_;
  commandedData_.gripperOpening = 1.0;

  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("RequestedState", requestedState_);
  datastore().make_call("getPostureTask",
                        [this]() -> mc_tasks::PostureTaskPtr
                        {
                          if(postureTask)
                          {
                            return postureTask;
                          }
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
  setupRosInterface();

  mc_rtc::log::success("[ExplicitCompCtrl] Init done ");
}

ExplicitCompCtrl::~ExplicitCompCtrl()
{
  stopRosInterface();
}

bool ExplicitCompCtrl::run()
{
  applyPendingCommand();
  updateInitialAutoTransition();
  if(measuredPublisher_ && runCounter_ % publishDecimation_ == 0)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = packRobotData(collectMeasuredData());
    measuredPublisher_->publish(msg);
  }
  ++runCounter_;

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

  if(config_.has("gripper"))
  {
    mc_rtc::Configuration gripper_config = config_("gripper");
    if(gripper_config.has("safety"))
    {
      mc_rtc::Configuration safety_config = gripper_config("safety");
      if(safety_config.has("actualCommandDiffTrigger"))
      {
        safety_config.add("actualCommandDiffTrigger",
                          mc_rtc::constants::toRad(static_cast<double>(safety_config("actualCommandDiffTrigger"))));
      }
      if(safety_config.has("releaseSafetyOffset"))
      {
        safety_config.add("releaseSafetyOffset",
                          mc_rtc::constants::toRad(static_cast<double>(safety_config("releaseSafetyOffset"))));
      }
      gripper_config.add("safety", safety_config);
    }
    robot().gripper("gripper").configure(gripper_config);
  }

  requestState("Initial");
}

void ExplicitCompCtrl::switchToInitialState()
{
  auto basePostureTask = getPostureTask(robot().name());
  RobotDataMessage command;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    command = commandedData_;
  }

  solver().removeTask(eeTask);
  eeTask.reset();

  solver().removeTask(postureTask);
  postureTask.reset();

  solver().removeTask(basePostureTask);
  if(basePostureTask)
  {
    basePostureTask->reset();
    basePostureTask->setGains(1.0, 2.0);
    basePostureTask->target(postureTargetMap(command.posture));
    solver().addTask(basePostureTask);
  }

  datastore().assign<std::string>("ControlMode", "Position");
  initialConvergenceCount_ = 0;
  requestState("Initial");
}

void ExplicitCompCtrl::switchToCompliantState()
{
  if(!datastore().call<bool>("EF_Estimator::isActive"))
  {
    datastore().call("EF_Estimator::toggleActive");
  }

  auto basePostureTask = getPostureTask(robot().name());
  RobotDataMessage command;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    command = commandedData_;
  }
  solver().removeTask(basePostureTask);

  solver().removeTask(postureTask);
  postureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1.0, 1.0);
  postureTask->reset();
  postureTask->setGains(1.0, 2.0);
  postureTask->target(postureTargetMap(command.posture));
  applyPostureCompliance();
  solver().addTask(postureTask);

  solver().removeTask(eeTask);
  eeTask = std::make_shared<mc_tasks::CompliantEndEffectorTask>(tool_frame, robots(), robot().robotIndex(), 100.0, 1e4);
  eeTask->positionTask->setGains(100.0, 20.0);
  eeTask->orientationTask->setGains(100.0, 20.0);
  applyCommandToTasks(command);
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
  postureComplianceCommand_.fill(gamma);
  applyPostureCompliance();
}

double ExplicitCompCtrl::postureCompliance() const
{
  return postureCompliance_;
}

void ExplicitCompCtrl::endEffectorCompliance(double gamma)
{
  endEffectorCompliance_ = gamma;
  endEffectorComplianceCommand_.fill(gamma);
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
      {"Controller", "ExplicitCompliance"},
      mc_rtc::gui::Label("Current mode", [this]() { return datastore().get<std::string>("ControlMode"); }),
      mc_rtc::gui::Button("Switch to initial state", [this]() { requestState("Initial"); }),
      mc_rtc::gui::Button("Switch to compliant state", [this]() { requestState("Compliant"); }));

  gui()->addElement({"Controller", "ExplicitCompliance", "Posture"},
                    mc_rtc::gui::NumberInput(
                        "Compliance scalar", [this]() { return postureCompliance(); },
                        [this](double gamma) { postureCompliance(gamma); }),
                    mc_rtc::gui::NumberSlider(
                        "Compliance scalar slider", [this]() { return postureCompliance(); },
                        [this](double gamma) { postureCompliance(gamma); }, 0.0, 1.2));

  gui()->addElement({"Controller", "ExplicitCompliance", "EndEffector"},
                    mc_rtc::gui::NumberInput(
                        "Compliance scalar", [this]() { return endEffectorCompliance(); },
                        [this](double gamma) { endEffectorCompliance(gamma); }),
                    mc_rtc::gui::NumberSlider(
                        "Compliance scalar slider", [this]() { return endEffectorCompliance(); },
                        [this](double gamma) { endEffectorCompliance(gamma); }, 0.0, 1.2));
}

void ExplicitCompCtrl::applyPostureCompliance()
{
  if(postureTask)
  {
    Eigen::VectorXd compliance(postureComplianceCommand_.size());
    for(size_t i = 0; i < postureComplianceCommand_.size(); ++i)
    {
      compliance(static_cast<Eigen::Index>(i)) = postureComplianceCommand_[i];
    }
    postureTask->makeCompliant(compliance);
    postureComplianceCurrent_ = postureComplianceCommand_;
  }
}

void ExplicitCompCtrl::applyEndEffectorCompliance()
{
  if(eeTask)
  {
    Eigen::Vector6d compliance;
    for(size_t i = 0; i < endEffectorComplianceCommand_.size(); ++i)
    {
      compliance(static_cast<Eigen::Index>(i)) = endEffectorComplianceCommand_[i];
    }
    eeTask->setComplianceVector(compliance);
    endEffectorComplianceCurrent_ = endEffectorComplianceCommand_;
  }
}

void ExplicitCompCtrl::updateInitialAutoTransition()
{
  if(requestedState() != "Initial" || datastore().get<std::string>("ControlMode") != "Position")
  {
    initialConvergenceCount_ = 0;
    return;
  }

  RobotDataMessage command;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    command = commandedData_;
  }

  const auto measured = measuredPostureArray();
  constexpr double threshold_rad = 10.0 * mc_rtc::constants::PI / 180.0;
  bool converged = true;
  for(size_t i = 0; i < measured.size(); ++i)
  {
    if(std::abs(measured[i] - command.posture[i]) >= threshold_rad)
    {
      converged = false;
      break;
    }
  }

  if(!converged)
  {
    initialConvergenceCount_ = 0;
    return;
  }

  ++initialConvergenceCount_;
  if(initialConvergenceCount_ >= 1000)
  {
    requestState("Compliant");
    initialConvergenceCount_ = 0;
  }
}

void ExplicitCompCtrl::setupRosInterface()
{
  rosContext_ = std::make_shared<rclcpp::Context>();
  rosContext_->init(0, nullptr);

  rclcpp::NodeOptions options;
  options.context(rosContext_);
  rosNode_ = std::make_shared<rclcpp::Node>("explicit_compliance_controller", options);
  rosCallbackGroup_ = rosNode_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = rosCallbackGroup_;
  commandSubscriber_ = rosNode_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "robot_command_data", rclcpp::QoS(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { handleCommandMessage(msg); }, sub_options);
  measuredPublisher_ =
      rosNode_->create_publisher<std_msgs::msg::Float64MultiArray>("robot_measured_data", rclcpp::QoS(1));

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = rosContext_;
  rosExecutor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  rosExecutor_->add_node(rosNode_);
  rosSpinThread_ = std::thread([this]() { rosExecutor_->spin(); });
}

void ExplicitCompCtrl::stopRosInterface()
{
  if(rosExecutor_)
  {
    rosExecutor_->cancel();
  }
  if(rosSpinThread_.joinable())
  {
    rosSpinThread_.join();
  }
  if(rosNode_ && rosExecutor_)
  {
    rosExecutor_->remove_node(rosNode_);
  }
  commandSubscriber_.reset();
  measuredPublisher_.reset();
  rosNode_.reset();
  rosExecutor_.reset();
  rosCallbackGroup_.reset();
  if(rosContext_)
  {
    rosContext_->shutdown("ExplicitCompCtrl shutdown");
    rosContext_.reset();
  }
}

void ExplicitCompCtrl::handleCommandMessage(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  RobotDataMessage unpacked;
  if(!unpackRobotData(msg->data, unpacked))
  {
    mc_rtc::log::warning("[ExplicitCompCtrl] robot_command_data size mismatch: expected 28, got {}", msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(commandMutex_);
  if(hasPendingCommand_)
  {
    mc_rtc::log::warning("[ExplicitCompCtrl] Overwriting pending robot_command_data before applyPendingCommand().");
  }
  commandedData_ = unpacked;
  hasPendingCommand_ = true;
}

void ExplicitCompCtrl::applyPendingCommand()
{
  RobotDataMessage local_command;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    if(!hasPendingCommand_)
    {
      return;
    }
    local_command = commandedData_;
    hasPendingCommand_ = false;
  }

  postureComplianceCommand_ = local_command.postureCompliance;
  endEffectorComplianceCommand_ = local_command.eeCompliance;
  applyCommandToTasks(local_command);
  applyPostureCompliance();
  applyEndEffectorCompliance();

  auto basePostureTask = getPostureTask(robot().name());
  if(basePostureTask && datastore().get<std::string>("ControlMode") == "Position")
  {
    basePostureTask->target(postureTargetMap(local_command.posture));
  }
}

void ExplicitCompCtrl::applyCommandToTasks(const RobotDataMessage & command)
{
  if(postureTask)
  {
    postureTask->target(postureTargetMap(command.posture));
  }

  if(eeTask)
  {
    eeTask->positionTask->position(
        Eigen::Vector3d(command.eePosition[0], command.eePosition[1], command.eePosition[2]));
    Eigen::Quaterniond quat(command.eeQuaternion[0], command.eeQuaternion[1], command.eeQuaternion[2],
                            command.eeQuaternion[3]);
    quat.normalize();
    eeTask->orientationTask->orientation(quat.toRotationMatrix());
  }

  if(!robot().grippers().empty())
  {
    robot().grippers().front().get().setTargetOpening(command.gripperOpening);
  }
}

ExplicitCompCtrl::RobotDataMessage ExplicitCompCtrl::collectMeasuredData() const
{
  RobotDataMessage measured;
  const auto measured_pose = measuredEndEffectorPose();
  const auto measured_quat = Eigen::Quaterniond(measured_pose.rotation());
  measured.eePosition = {measured_pose.translation().x(), measured_pose.translation().y(),
                         measured_pose.translation().z()};
  measured.eeQuaternion = {measured_quat.w(), measured_quat.x(), measured_quat.y(), measured_quat.z()};
  measured.posture = measuredPostureArray();
  measured.eeCompliance = endEffectorComplianceCurrent_;
  measured.postureCompliance = postureComplianceCurrent_;
  if(!robot().grippers().empty())
  {
    measured.gripperOpening = robot().grippers().front().get().opening();
  }
  return measured;
}

std::vector<double> ExplicitCompCtrl::packRobotData(const RobotDataMessage & data) const
{
  std::vector<double> packed;
  packed.reserve(28);
  packed.insert(packed.end(), data.eePosition.begin(), data.eePosition.end());
  packed.insert(packed.end(), data.eeQuaternion.begin(), data.eeQuaternion.end());
  packed.insert(packed.end(), data.posture.begin(), data.posture.end());
  packed.insert(packed.end(), data.eeCompliance.begin(), data.eeCompliance.end());
  packed.insert(packed.end(), data.postureCompliance.begin(), data.postureCompliance.end());
  packed.push_back(data.gripperOpening);
  return packed;
}

bool ExplicitCompCtrl::unpackRobotData(const std::vector<double> & data, RobotDataMessage & unpacked) const
{
  if(data.size() != 28)
  {
    return false;
  }
  size_t offset = 0;
  for(size_t i = 0; i < unpacked.eePosition.size(); ++i)
  {
    unpacked.eePosition[i] = data[offset++];
  }
  for(size_t i = 0; i < unpacked.eeQuaternion.size(); ++i)
  {
    unpacked.eeQuaternion[i] = data[offset++];
  }
  for(size_t i = 0; i < unpacked.posture.size(); ++i)
  {
    unpacked.posture[i] = data[offset++];
  }
  for(size_t i = 0; i < unpacked.eeCompliance.size(); ++i)
  {
    unpacked.eeCompliance[i] = data[offset++];
  }
  for(size_t i = 0; i < unpacked.postureCompliance.size(); ++i)
  {
    unpacked.postureCompliance[i] = data[offset++];
  }
  unpacked.gripperOpening = data[offset++];
  return true;
}

std::map<std::string, std::vector<double>> ExplicitCompCtrl::postureTargetMap(
    const std::array<double, 7> & posture) const
{
  std::map<std::string, std::vector<double>> target;
  for(size_t i = 0; i < postureJointNames_.size(); ++i)
  {
    target[postureJointNames_[i]] = {posture[i]};
  }
  return target;
}

std::array<double, 7> ExplicitCompCtrl::measuredPostureArray() const
{
  std::array<double, 7> measured = {};
  const auto posture_vector = rbd::dofToVector(robot().mb(), robot().mbc().q);
  for(size_t i = 0; i < measured.size() && i < static_cast<size_t>(posture_vector.size()); ++i)
  {
    measured[i] = posture_vector[static_cast<Eigen::Index>(i)];
  }
  return measured;
}

sva::PTransformd ExplicitCompCtrl::measuredEndEffectorPose() const
{
  if(robot().hasFrame(tool_frame))
  {
    return robot().frame(tool_frame).position();
  }
  return robot().bodyPosW(tool_frame);
}
