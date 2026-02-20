"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import cereal.messaging as messaging
from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V_ADVISORY

AdvisoryState = custom.LongitudinalPlanSP.SmartCruiseControl.AdvisoryState

ACTIVE_STATES = (AdvisoryState.active, )
ENABLED_STATES = (AdvisoryState.enabled, AdvisoryState.overriding, *ACTIVE_STATES)


class SmartCruiseControlAdvisory:
  """Advisory speed controller for roundabouts and curves with OSM advisory speed limits.

  Reads advisory speed limits from liveMapDataSP (sourced from mapd's MapAdvisorySpeedLimit param)
  and applies them as a target speed when the advisory speed is lower than the current cruise speed.
  """
  v_target: float = 0.
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.enabled = self.params.get_bool("SmartCruiseControlAdvisory")
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.state = AdvisoryState.disabled
    self.v_cruise = 0.
    self.advisory_speed_limit = 0.
    self.advisory_speed_limit_valid = False

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V_ADVISORY)
    return V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return min(self.a_ego, 0.) if self.is_active else 0.

  def update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlAdvisory")

  def update_calculations(self, sm: messaging.SubMaster) -> None:
    map_data = sm['liveMapDataSP']
    self.advisory_speed_limit_valid = map_data.advisorySpeedLimitValid
    # When advisory becomes invalid, speed resets to 0 immediately. The state machine
    # transitions active -> enabled on the same frame, so output_v_target returns to
    # V_CRUISE_UNSET within one cycle. This is safe because the longitudinal planner
    # picks min(targets) and V_CRUISE_UNSET is the highest possible value.
    self.advisory_speed_limit = map_data.advisorySpeedLimit if self.advisory_speed_limit_valid else 0.
    self.v_target = self.advisory_speed_limit

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ACTIVE, OVERRIDING
    if self.state != AdvisoryState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = AdvisoryState.disabled
      elif self.long_override:
        self.state = AdvisoryState.overriding
      else:
        # ENABLED
        if self.state == AdvisoryState.enabled:
          if self.advisory_speed_limit_valid and self.v_cruise > self.advisory_speed_limit > 0:
            self.state = AdvisoryState.active

        # ACTIVE
        elif self.state == AdvisoryState.active:
          if not self.advisory_speed_limit_valid or self.advisory_speed_limit <= 0:
            self.state = AdvisoryState.enabled
          elif self.v_cruise <= self.advisory_speed_limit:
            self.state = AdvisoryState.enabled

        # OVERRIDING
        elif self.state == AdvisoryState.overriding:
          if not self.long_override:
            if self.advisory_speed_limit_valid and self.v_cruise > self.advisory_speed_limit > 0:
              self.state = AdvisoryState.active
            else:
              self.state = AdvisoryState.enabled

    # DISABLED
    elif self.state == AdvisoryState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = AdvisoryState.overriding
        else:
          self.state = AdvisoryState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool,
             v_ego: float, a_ego: float, v_cruise: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise = v_cruise

    self.update_params()
    self.update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
