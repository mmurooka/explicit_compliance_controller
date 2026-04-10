#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "api.h"

struct ExplicitCompCtrl_DLLAPI ExplicitCompCtrl : public mc_control::fsm::Controller
{
  ExplicitCompCtrl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  ~ExplicitCompCtrl() override;

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void switchToInitialState();
  void switchToCompliantState();
  void requestState(const std::string & state);
  const std::string & requestedState() const;

  void postureCompliance(double gamma);
  double postureCompliance() const;

  void endEffectorCompliance(double gamma);
  double endEffectorCompliance() const;

  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> eeTask;
  std::shared_ptr<mc_tasks::CompliantPostureTask> postureTask;

  std::string tool_frame;

private:
  struct RobotDataMessage
  {
    std::array<double, 3> eePosition = {};
    std::array<double, 4> eeQuaternion = {};
    std::array<double, 7> posture = {};
    std::array<double, 6> eeCompliance = {};
    std::array<double, 7> postureCompliance = {};
    double gripperOpening = 0.0;
  };

  enum class PendingStateTarget
  {
    None,
    Initial,
    Compliant,
  };

  struct PendingStateTransition
  {
    PendingStateTarget target = PendingStateTarget::None;
    bool active = false;
    bool completed = false;
    bool success = false;
    std::string message;
  };

  void addGui();
  void setupRosInterface();
  void stopRosInterface();
  void handleCommandMessage(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void applyPendingCommand();
  void applyCommandToTasks(const RobotDataMessage & command);
  void updateStateServiceRequests();
  void finishStateServiceRequest(bool success, const std::string & message);
  bool isInInitialMode() const;
  bool isInCompliantMode() const;
  bool isInitialTargetConverged() const;
  void applyPostureTarget(const std::array<double, 7> & posture);
  void applyPostureCompliance();
  void applyEndEffectorCompliance();
  void handlePendingStateTransitionService(PendingStateTarget target,
                                           const std::string & state_name,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleGoToInitial(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleGoToCompliant(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  rcl_interfaces::msg::SetParametersResult handleParameters(const std::vector<rclcpp::Parameter> & parameters);
  RobotDataMessage collectMeasuredData() const;
  std::vector<double> packRobotData(const RobotDataMessage & data) const;
  bool unpackRobotData(const std::vector<double> & data, RobotDataMessage & unpacked) const;
  std::map<std::string, std::vector<double>> postureTargetMap(const std::array<double, 7> & posture) const;
  std::array<double, 7> measuredPostureArray() const;
  sva::PTransformd measuredEndEffectorPose() const;

  mc_rtc::Configuration config_;
  std::string requestedState_ = "Initial";
  double postureCompliance_ = 1.0;
  double endEffectorCompliance_ = 1.0;
  std::array<double, 7> postureComplianceCommand_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 6> endEffectorComplianceCommand_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> postureComplianceCurrent_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 6> endEffectorComplianceCurrent_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  RobotDataMessage commandedData_;
  mutable std::mutex commandMutex_;
  bool hasPendingCommand_ = false;
  std::vector<std::string> postureJointNames_ = {"joint_1", "joint_2", "joint_3", "joint_4",
                                                 "joint_5", "joint_6", "joint_7"};
  std::shared_ptr<rclcpp::Node> rosNode_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr measuredPublisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commandSubscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goToInitialService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goToCompliantService_;
  rclcpp::CallbackGroup::SharedPtr rosCallbackGroup_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterCallbackHandle_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor_;
  rclcpp::Context::SharedPtr rosContext_;
  std::thread rosSpinThread_;
  size_t publishDecimation_ = 10;
  size_t runCounter_ = 0;
  size_t initialConvergenceCount_ = 0;
  mutable std::mutex stateServiceMutex_;
  std::condition_variable stateServiceCv_;
  PendingStateTransition pendingStateTransition_;
};
