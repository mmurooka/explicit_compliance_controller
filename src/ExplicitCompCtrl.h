#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>

#include "api.h"

struct ExplicitCompCtrl_DLLAPI ExplicitCompCtrl : public mc_control::fsm::Controller
{
  ExplicitCompCtrl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

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

  std::map<std::string, std::vector<double>> postureHome;

  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> eeTask;
  std::shared_ptr<mc_tasks::CompliantPostureTask> postureTask;

  std::string tool_frame;

private:
  void addGui();
  void applyPostureCompliance();
  void applyEndEffectorCompliance();

  mc_rtc::Configuration config_;
  std::string requestedState_ = "Initial";
  double postureCompliance_ = 1.0;
  double endEffectorCompliance_ = 1.0;
};
