"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest
import cereal.messaging as messaging
from cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V_ADVISORY
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.advisory_controller import (
  SmartCruiseControlAdvisory, ROUNDABOUT_DECEL, ROUNDABOUT_BUFFER,
)

AdvisoryState = custom.LongitudinalPlanSP.SmartCruiseControl.AdvisoryState


def generate_liveMapDataSP(advisory_speed_limit: float = 0., advisory_valid: bool = False,
                           is_roundabout: bool = False, roundabout_distance: float = 0.):
  msg = messaging.new_message('liveMapDataSP')
  msg.liveMapDataSP.advisorySpeedLimitValid = advisory_valid
  msg.liveMapDataSP.advisorySpeedLimit = advisory_speed_limit
  msg.liveMapDataSP.isRoundabout = is_roundabout
  msg.liveMapDataSP.roundaboutDistance = roundabout_distance
  return msg


class TestSmartCruiseControlAdvisory:

  def setup_method(self):
    self.scc_a = SmartCruiseControlAdvisory()

    map_data = generate_liveMapDataSP()
    self.sm = {'liveMapDataSP': map_data.liveMapDataSP}

  def _update_map_data(self, advisory_speed: float, advisory_valid: bool = True,
                       is_roundabout: bool = False, roundabout_distance: float = 0.):
    msg = generate_liveMapDataSP(advisory_speed, advisory_valid, is_roundabout, roundabout_distance)
    self.sm['liveMapDataSP'] = msg.liveMapDataSP

  def test_initial_state(self):
    assert self.scc_a.state == AdvisoryState.disabled
    assert not self.scc_a.is_active
    assert self.scc_a.output_v_target == V_CRUISE_UNSET
    assert self.scc_a.output_a_target == 0.

  def test_system_disabled(self):
    self.scc_a.enabled = False  # Simulate toggle off

    for _ in range(int(10. / DT_MDL)):
      self.scc_a.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_a.state == AdvisoryState.disabled
    assert not self.scc_a.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_a.update(self.sm, False, False, 0., 0., 0.)
    assert self.scc_a.state == AdvisoryState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_a.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_a.state == AdvisoryState.enabled

  def test_transition_enabled_to_active_with_advisory_speed(self):
    advisory_speed = 30 * CV.KPH_TO_MS  # 30 km/h advisory
    v_cruise = 50 * CV.KPH_TO_MS  # 50 km/h cruise
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # First transition to enabled
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.enabled

    # Now should transition to active since v_cruise > advisory_speed
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.active
    assert self.scc_a.is_active

  def test_active_returns_advisory_speed_as_target(self):
    advisory_speed = 25 * CV.KPH_TO_MS  # 25 km/h advisory (roundabout)
    v_cruise = 60 * CV.KPH_TO_MS
    v_ego = 55 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.is_active
    assert self.scc_a.output_v_target == pytest.approx(max(advisory_speed, MIN_V_ADVISORY), abs=1e-4)

  def test_active_respects_min_v_advisory(self):
    advisory_speed = 20 * CV.KPH_TO_MS  # 20 km/h - below MIN_V_ADVISORY (30 km/h)
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.is_active
    assert self.scc_a.output_v_target == MIN_V_ADVISORY

  def test_transition_active_to_enabled_when_advisory_cleared(self):
    advisory_speed = 30 * CV.KPH_TO_MS
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.active

    # Clear advisory speed
    self._update_map_data(0., advisory_valid=False)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active
    assert self.scc_a.output_v_target == V_CRUISE_UNSET

  def test_transition_active_to_enabled_when_cruise_below_advisory(self):
    advisory_speed = 30 * CV.KPH_TO_MS
    v_cruise_high = 50 * CV.KPH_TO_MS
    v_cruise_low = 25 * CV.KPH_TO_MS
    v_ego = 28 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise_high)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise_high)
    assert self.scc_a.state == AdvisoryState.active

    # Now cruise is below advisory - should go to enabled
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise_low)
    assert self.scc_a.state == AdvisoryState.enabled

  def test_overriding_state(self):
    # disabled -> enabled (override)
    self.scc_a.update(self.sm, True, True, 0., 0., 0.)
    assert self.scc_a.state == AdvisoryState.overriding

    # overriding -> enabled when override ends
    self.scc_a.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_a.state == AdvisoryState.enabled

  def test_not_active_when_advisory_higher_than_cruise(self):
    advisory_speed = 60 * CV.KPH_TO_MS  # advisory is higher than cruise
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # disabled -> enabled
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    # Should stay enabled, not active - advisory is above cruise
    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active

  def test_disable_on_long_disabled(self):
    advisory_speed = 30 * CV.KPH_TO_MS
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # Go to active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.active

    # Disable longitudinal
    self.scc_a.update(self.sm, False, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.disabled

  def test_a_target_clamped_to_non_positive_when_active(self):
    advisory_speed = 30 * CV.KPH_TO_MS
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS
    a_ego_positive = 1.5  # accelerating

    self._update_map_data(advisory_speed)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, a_ego_positive, v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, a_ego_positive, v_cruise)

    assert self.scc_a.is_active
    # a_target should be clamped to 0 when a_ego is positive
    assert self.scc_a.output_a_target == 0.

    # With negative a_ego, should pass through
    a_ego_negative = -0.5
    self.scc_a.update(self.sm, True, False, v_ego, a_ego_negative, v_cruise)
    assert self.scc_a.output_a_target == a_ego_negative

  def test_boundary_cruise_equals_advisory(self):
    advisory_speed = 50 * CV.KPH_TO_MS
    v_cruise = 50 * CV.KPH_TO_MS  # exactly equal
    v_ego = 48 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed)

    # disabled -> enabled
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    # Should NOT be active when cruise == advisory (requires cruise > advisory)
    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active

  def test_advisory_value_change_while_active(self):
    advisory_speed_1 = 30 * CV.KPH_TO_MS
    advisory_speed_2 = 40 * CV.KPH_TO_MS
    v_cruise = 60 * CV.KPH_TO_MS
    v_ego = 55 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed_1)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.active
    assert self.scc_a.output_v_target == pytest.approx(advisory_speed_1, abs=1e-4)

    # Advisory changes to a higher value while still active
    self._update_map_data(advisory_speed_2)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.active
    assert self.scc_a.output_v_target == pytest.approx(advisory_speed_2, abs=1e-4)

  # --- Roundabout fallback tests (distance-based) ---

  def _brake_distance(self, v_ego: float) -> float:
    """Helper: SUVAT braking distance from v_ego to MIN_V_ADVISORY at ROUNDABOUT_DECEL."""
    return (v_ego ** 2 - MIN_V_ADVISORY ** 2) / (2 * abs(ROUNDABOUT_DECEL))

  def test_roundabout_fallback_when_within_braking_distance(self):
    """Roundabout within braking distance + buffer should activate at 30 km/h."""
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 50 * CV.KPH_TO_MS
    # Distance within braking range: brake_dist + buffer - 10m
    dist = self._brake_distance(v_ego) + ROUNDABOUT_BUFFER - 10

    self._update_map_data(0., advisory_valid=False, is_roundabout=True, roundabout_distance=dist)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.state == AdvisoryState.active
    assert self.scc_a.is_active
    assert self.scc_a.output_v_target == pytest.approx(MIN_V_ADVISORY, abs=1e-4)

  def test_roundabout_no_activation_when_far_away(self):
    """Roundabout beyond braking distance + buffer should NOT activate."""
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 50 * CV.KPH_TO_MS
    # Distance beyond braking range: brake_dist + buffer + 50m
    dist = self._brake_distance(v_ego) + ROUNDABOUT_BUFFER + 50

    self._update_map_data(0., advisory_valid=False, is_roundabout=True, roundabout_distance=dist)

    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active
    assert self.scc_a.output_v_target == V_CRUISE_UNSET

  def test_roundabout_higher_speed_needs_more_distance(self):
    """At higher speeds, braking should start earlier (larger activation distance)."""
    v_cruise_slow = 50 * CV.KPH_TO_MS
    v_ego_slow = 50 * CV.KPH_TO_MS
    v_cruise_fast = 100 * CV.KPH_TO_MS
    v_ego_fast = 100 * CV.KPH_TO_MS

    brake_dist_slow = self._brake_distance(v_ego_slow)
    brake_dist_fast = self._brake_distance(v_ego_fast)

    # Fast needs significantly more distance
    assert brake_dist_fast > brake_dist_slow * 2

    # At a distance that activates for fast but NOT for slow
    mid_dist = brake_dist_slow + ROUNDABOUT_BUFFER + 30  # just outside slow range

    # Slow speed: should NOT activate
    self._update_map_data(0., advisory_valid=False, is_roundabout=True, roundabout_distance=mid_dist)
    self.scc_a.update(self.sm, True, False, v_ego_slow, 0., v_cruise_slow)
    self.scc_a.update(self.sm, True, False, v_ego_slow, 0., v_cruise_slow)
    assert not self.scc_a.is_active

    # Fast speed: should activate at same distance
    self.scc_a.update(self.sm, True, False, v_ego_fast, 0., v_cruise_fast)
    assert self.scc_a.is_active

  def test_roundabout_advisory_takes_precedence_over_fallback(self):
    """If both advisory speed AND roundabout, use the actual advisory speed."""
    advisory_speed = 25 * CV.KPH_TO_MS  # 25 km/h explicit advisory
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(advisory_speed, advisory_valid=True, is_roundabout=True, roundabout_distance=80.)

    # disabled -> enabled -> active
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.is_active
    # Should use the explicit advisory speed (clamped to MIN_V_ADVISORY since 25 < 30)
    assert self.scc_a.output_v_target == pytest.approx(MIN_V_ADVISORY, abs=1e-4)

  def test_no_roundabout_no_advisory_stays_enabled(self):
    """Without roundabout and without advisory, should NOT become active."""
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    # No advisory, no roundabout
    self._update_map_data(0., advisory_valid=False, is_roundabout=False)

    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active
    assert self.scc_a.output_v_target == V_CRUISE_UNSET

  def test_roundabout_deactivates_when_leaving(self):
    """When roundabout distance goes to 0, should transition back to enabled."""
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 35 * CV.KPH_TO_MS
    dist = 60.  # close enough to activate

    # Approaching roundabout
    self._update_map_data(0., advisory_valid=False, is_roundabout=True, roundabout_distance=dist)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.active

    # Left roundabout area
    self._update_map_data(0., advisory_valid=False, is_roundabout=False)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active

  def test_roundabout_not_active_when_cruise_below_fallback(self):
    """If cruise is already at/below 30 km/h, roundabout fallback should not activate."""
    v_cruise = 25 * CV.KPH_TO_MS  # below MIN_V_ADVISORY
    v_ego = 20 * CV.KPH_TO_MS

    self._update_map_data(0., advisory_valid=False, is_roundabout=True, roundabout_distance=50.)

    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    # v_cruise <= MIN_V_ADVISORY, so should stay enabled (no need to slow down further)
    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active

  def test_roundabout_zero_distance_no_activation(self):
    """If roundabout detected but distance is 0, should not activate (no geometry data)."""
    v_cruise = 50 * CV.KPH_TO_MS
    v_ego = 45 * CV.KPH_TO_MS

    self._update_map_data(0., advisory_valid=False, is_roundabout=True, roundabout_distance=0.)

    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)
    self.scc_a.update(self.sm, True, False, v_ego, 0., v_cruise)

    assert self.scc_a.state == AdvisoryState.enabled
    assert not self.scc_a.is_active
