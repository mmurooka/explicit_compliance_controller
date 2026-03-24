#include "ExplicitCompCtrl_Compliant.h"

#include "../ExplicitCompCtrl.h"

void ExplicitCompCtrl_Compliant::configure(const mc_rtc::Configuration & config) {}

void ExplicitCompCtrl_Compliant::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ExplicitCompCtrl &>(ctl_);
  ctl.switchToCompliantState();
}

bool ExplicitCompCtrl_Compliant::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ExplicitCompCtrl &>(ctl_);
  if(ctl.requestedState() == "Initial")
  {
    output("GoToInitial");
    return true;
  }
  return false;
}

void ExplicitCompCtrl_Compliant::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ExplicitCompCtrl &>(ctl_);
}

EXPORT_SINGLE_STATE("ExplicitCompCtrl_Compliant", ExplicitCompCtrl_Compliant)
