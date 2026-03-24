#include "ExplicitCompCtrl_Initial.h"

#include "../ExplicitCompCtrl.h"

void ExplicitCompCtrl_Initial::configure(const mc_rtc::Configuration & config) {}

void ExplicitCompCtrl_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ExplicitCompCtrl &>(ctl_);
  ctl.switchToInitialState();
}

bool ExplicitCompCtrl_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ExplicitCompCtrl &>(ctl_);
  if(ctl.requestedState() == "Compliant")
  {
    output("GoToCompliant");
    return true;
  }
  return false;
}

void ExplicitCompCtrl_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ExplicitCompCtrl &>(ctl_);
}

EXPORT_SINGLE_STATE("ExplicitCompCtrl_Initial", ExplicitCompCtrl_Initial)
