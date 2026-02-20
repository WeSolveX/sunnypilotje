"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

import pytest
from pytest_mock import MockerFixture

from cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE, SUDDEN_LIMIT_DEBOUNCE_TIME

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker: MockerFixture):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker: MockerFixture):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  gps_data = create_mock({
    'unixTimestampMillis': time.monotonic() * 1e3,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  return sm_mock


def make_debounce_sm(mocker: MockerFixture, current_time: float, ahead_distance: float = 0.):
  """Create a minimal sm mock for _calculate_map_data_limits calls with controlled time."""
  gps_data = create_mock({
    'unixTimestampMillis': current_time * 1e3,
  }, mocker)
  map_data = create_mock({
    'speedLimitAheadDistance': ahead_distance,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'liveMapDataSP': map_data,
    'gpsLocation': gps_data,
  }[key]
  return sm_mock


parametrized_policies = pytest.mark.parametrize(
  "policy, sm_key, function_key", [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  ids=lambda val: val.name if hasattr(val, 'name') else str(val)
)


@pytest.mark.parametrize("resolver_class", [SpeedLimitResolver])
class TestSpeedLimitResolverValidation:

  @pytest.mark.parametrize("policy", list(Policy), ids=lambda policy: policy.name)
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Pre-set accepted map limit to bypass debounce (this test validates policy resolution)
    resolver._accepted_map_limit = sm_mock['liveMapDataSP'].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Pre-set accepted map limit to bypass debounce (this test validates combined policy)
    resolver._accepted_map_limit = sm_mock['liveMapDataSP'].speedLimit

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Pre-set accepted map limit to bypass debounce (this test validates parsing)
    resolver._accepted_map_limit = sm_mock['liveMapDataSP'].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @pytest.mark.parametrize("policy", list(Policy), ids=lambda policy: policy.name)
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker: MockerFixture):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)

    # Pre-set accepted map limit to bypass debounce
    resolver._accepted_map_limit = sm_mock['liveMapDataSP'].speedLimit

    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @pytest.mark.parametrize("policy", list(Policy), ids=lambda policy: policy.name)
  def test_old_map_data_ignored(self, resolver_class, policy, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = mocker.MagicMock()
    sm_mock['gpsLocation'].unixTimestampMillis = (time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE) * 1e3
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.


class TestSuddenSpeedLimitDebounce:
  """Tests for the 3-second debounce of sudden (not seen ahead) speed limits."""

  LIMIT_HIGH = 30.56   # ~110 km/h in m/s
  LIMIT_LOW = 13.89    # ~50 km/h in m/s
  LIMIT_MID = 22.22    # ~80 km/h in m/s

  def _make_resolver_with_accepted_limit(self, limit: float) -> SpeedLimitResolver:
    """Create a resolver with an already-accepted map speed limit."""
    resolver = SpeedLimitResolver()
    resolver.policy = Policy.map_data_only
    resolver.v_ego = 30.
    resolver._accepted_map_limit = limit
    return resolver

  def test_sudden_lower_limit_debounced_for_3s(self, mocker: MockerFixture):
    """Sudden drop 110->50: held at 110 until 3s elapsed, then 50 accepted."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_HIGH)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # t=0: sudden drop to 50 -> debounce starts, output stays at 110
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH

    # t=1.5s: still debouncing
    mock_monotonic.return_value = 1001.5
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH

    # t=3s: debounce complete -> 50 accepted
    mock_monotonic.return_value = 1003.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_LOW
    assert resolver._accepted_map_limit == self.LIMIT_LOW

  def test_sudden_higher_limit_debounced_for_3s(self, mocker: MockerFixture):
    """Sudden rise 50->110: held at 50 until 3s elapsed, then 110 accepted."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_LOW)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # t=0: sudden rise to 110 -> debounce starts
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_LOW

    # t=3s: debounce complete -> 110 accepted
    mock_monotonic.return_value = 1003.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH
    assert resolver._accepted_map_limit == self.LIMIT_HIGH

  def test_sudden_limit_disappears_before_3s_ignored(self, mocker: MockerFixture):
    """Transient 50 appears for 2s then vanishes -> ignored, stays at 110."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_HIGH)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # t=0: sudden drop to 50
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH

    # t=2s: limit disappears (goes to 0) -> pending state cleared
    mock_monotonic.return_value = 1002.0
    resolver._calculate_map_data_limits(sm, 0., 0.)
    assert resolver._pending_sudden_limit == 0.
    assert resolver._pending_sudden_ts == 0.

    # Original limit returns -> no debounce (matches accepted)
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH

  def test_limit_seen_ahead_lower_accepted_immediately(self, mocker: MockerFixture):
    """Lower limit seen as speedLimitAhead first -> no debounce when it becomes current."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_HIGH)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # Step 1: see 50 as upcoming limit (current stays at 110)
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, self.LIMIT_LOW)
    assert resolver._last_ahead_limit == self.LIMIT_LOW

    # Step 2: 50 becomes current limit -> accepted immediately (was seen ahead)
    mock_monotonic.return_value = 1001.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, 0.)
    assert resolver._accepted_map_limit == self.LIMIT_LOW
    # Lookahead may override limit_solutions, so check _accepted_map_limit directly

  def test_limit_seen_ahead_higher_accepted_immediately(self, mocker: MockerFixture):
    """Higher limit seen as speedLimitAhead first -> no debounce when it becomes current."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_LOW)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # Step 1: see 110 as upcoming limit (current stays at 50)
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, self.LIMIT_HIGH)
    assert resolver._last_ahead_limit == self.LIMIT_HIGH

    # Step 2: 110 becomes current limit -> accepted immediately
    mock_monotonic.return_value = 1001.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver._accepted_map_limit == self.LIMIT_HIGH

  def test_last_ahead_limit_cleared_when_ahead_drops_to_zero(self, mocker: MockerFixture):
    """_last_ahead_limit resets to 0 when speedLimitAhead becomes 0."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_HIGH)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # See ahead limit
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, self.LIMIT_LOW)
    assert resolver._last_ahead_limit == self.LIMIT_LOW

    # Ahead drops to 0 (current stays same) -> _last_ahead_limit cleared
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver._last_ahead_limit == 0.

  def test_rapid_oscillation_keeps_debouncing(self, mocker: MockerFixture):
    """Oscillating between 50 and 80 resets the timer each time."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_HIGH)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # t=0: sudden drop to 50
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH

    # t=2s: switch to 80 -> timer resets
    mock_monotonic.return_value = 1002.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_MID, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH
    assert resolver._pending_sudden_limit == self.LIMIT_MID

    # t=4s: only 2s since 80 appeared -> still debouncing
    mock_monotonic.return_value = 1004.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_MID, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH

    # t=5s: 3s since 80 appeared -> accepted
    mock_monotonic.return_value = 1005.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_MID, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_MID
    assert resolver._accepted_map_limit == self.LIMIT_MID

  def test_first_limit_after_restart_debounced(self, mocker: MockerFixture):
    """After process restart (all state=0), first limit is debounced for 3s."""
    resolver = SpeedLimitResolver()
    resolver.policy = Policy.map_data_only
    resolver.v_ego = 30.
    # _accepted_map_limit defaults to 0 (fresh start)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0)

    # t=0: first limit appears -> debounced (output is 0 since _accepted=0)
    mock_monotonic.return_value = 1000.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.

    # t=3s: accepted
    mock_monotonic.return_value = 1003.0
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, 0.)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH
    assert resolver._accepted_map_limit == self.LIMIT_HIGH

  def test_car_state_source_not_debounced(self, mocker: MockerFixture):
    """Car state speed limits bypass the debounce entirely."""
    resolver = SpeedLimitResolver()
    resolver.policy = Policy.car_state_only
    resolver.v_ego = 30.

    sm = setup_sm_mock(mocker)
    car_limit = sm['carStateSP'].speedLimit

    # Car state source goes directly through _get_from_car_state, no debounce
    resolver._get_from_car_state(sm)
    assert resolver.limit_solutions[SpeedLimitSource.car] == car_limit

  def test_lookahead_early_braking_with_debounced_input(self, mocker: MockerFixture):
    """Lookahead early braking still works correctly on debounced data."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_HIGH)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    sm = make_debounce_sm(mocker, current_time=1000.0, ahead_distance=50.)

    mock_monotonic.return_value = 1000.0

    # Current=110 (accepted, no debounce), ahead=50, close distance -> lookahead overrides to 50
    resolver._calculate_map_data_limits(sm, self.LIMIT_HIGH, self.LIMIT_LOW)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_LOW
    assert resolver.distance_solutions[SpeedLimitSource.map] == 50.

  def test_lookahead_early_acceleration_with_debounced_input(self, mocker: MockerFixture):
    """Lookahead early acceleration still works correctly on debounced data."""
    resolver = self._make_resolver_with_accepted_limit(self.LIMIT_LOW)

    mock_monotonic = mocker.patch('time.monotonic')
    mocker.patch('time.time', return_value=1000.0)
    # ahead_distance within lookahead range: min(max(50, 30*5), 250) = 150m
    sm = make_debounce_sm(mocker, current_time=1000.0, ahead_distance=100.)

    mock_monotonic.return_value = 1000.0

    # Current=50 (accepted), ahead=110, distance=100m <= 150m -> override to 110
    resolver._calculate_map_data_limits(sm, self.LIMIT_LOW, self.LIMIT_HIGH)
    assert resolver.limit_solutions[SpeedLimitSource.map] == self.LIMIT_HIGH
    assert resolver.distance_solutions[SpeedLimitSource.map] == 100.
