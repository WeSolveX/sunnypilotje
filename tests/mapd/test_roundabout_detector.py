"""
Tests for the RoundaboutDetector lateral distance filtering.

Verifies that roundabouts on parallel roads (e.g., next to a motorway)
are rejected, while roundabouts directly ahead or on an off-ramp are accepted.

Uses importlib to load the module directly, bypassing the openpilot package chain
that requires the full embedded environment.
"""
import importlib.util
import math
import os
import sys
import types

import pytest

# Load osm_map_data.py directly via importlib to bypass openpilot package imports.
# We stub out the dependencies that aren't needed for RoundaboutDetector's pure logic.
_MODULE_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'sunnypilot', 'mapd', 'live_map_data', 'osm_map_data.py')
_MODULE_PATH = os.path.normpath(_MODULE_PATH)


def _load_osm_map_data():
  """Load osm_map_data module with stubbed dependencies."""
  # Stub openpilot packages that the module imports but RoundaboutDetector doesn't use
  stub_modules = [
    'cereal', 'cereal.log',
    'openpilot', 'openpilot.common', 'openpilot.common.constants',
    'openpilot.common.params', 'openpilot.common.swaglog',
    'openpilot.sunnypilot', 'openpilot.sunnypilot.mapd',
    'openpilot.sunnypilot.mapd.live_map_data',
    'openpilot.sunnypilot.mapd.live_map_data.base_map_data',
    'openpilot.sunnypilot.navd', 'openpilot.sunnypilot.navd.helpers',
  ]

  saved = {}
  for mod_name in stub_modules:
    if mod_name in sys.modules:
      saved[mod_name] = sys.modules[mod_name]

  try:
    # Create stub modules with all attributes that osm_map_data.py imports
    for mod_name in stub_modules:
      stub = types.ModuleType(mod_name)
      sys.modules[mod_name] = stub

    # cereal.log needs a nested Status enum for OsmMapData (not used by RoundaboutDetector)
    log_stub = sys.modules['cereal.log']
    status_stub = types.SimpleNamespace(valid=1)
    llk_stub = types.SimpleNamespace(Status=status_stub)
    log_stub.LiveLocationKalman = llk_stub
    sys.modules['cereal'].log = log_stub

    # CV.KPH_TO_MS is needed for speed constants
    cv = types.SimpleNamespace(KPH_TO_MS=1.0 / 3.6)
    sys.modules['openpilot.common.constants'].CV = cv

    # Params class stub (used by OsmMapData, not by RoundaboutDetector)
    sys.modules['openpilot.common.params'].Params = type('Params', (), {
      '__init__': lambda self, *a, **kw: None,
      'get_bool': lambda self, *a, **kw: False,
      'get': lambda self, *a, **kw: None,
    })

    # cloudlog stub
    sys.modules['openpilot.common.swaglog'].cloudlog = types.SimpleNamespace(
      debug=lambda *a, **kw: None
    )

    # BaseMapData dummy base class
    sys.modules['openpilot.sunnypilot.mapd.live_map_data.base_map_data'].BaseMapData = type(
      'BaseMapData', (), {'__init__': lambda self: None}
    )

    # Coordinate stub
    sys.modules['openpilot.sunnypilot.navd.helpers'].Coordinate = type(
      'Coordinate', (), {'__init__': lambda self, *a: None}
    )

    spec = importlib.util.spec_from_file_location('osm_map_data', _MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
  finally:
    # Restore original modules
    for mod_name in stub_modules:
      if mod_name in saved:
        sys.modules[mod_name] = saved[mod_name]
      elif mod_name in sys.modules:
        del sys.modules[mod_name]


_osm = _load_osm_map_data()

# Import the actual classes/functions/constants we need to test
RoundaboutDetector = _osm.RoundaboutDetector
_bearing_to = _osm._bearing_to
_haversine_distance = _osm._haversine_distance
_is_ahead = _osm._is_ahead
MAX_LATERAL_OFFSET = _osm.MAX_LATERAL_OFFSET
ROUNDABOUT_BEHIND_THRESHOLD = _osm.ROUNDABOUT_BEHIND_THRESHOLD
ROUNDABOUT_EARLY_BRAKING_SECS = _osm.ROUNDABOUT_EARLY_BRAKING_SECS


def _offset_coordinate(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
  """Offset a lat/lon coordinate by meters north and east (approximate)."""
  new_lat = lat + north_m / 111000.0
  new_lon = lon + east_m / (111000.0 * math.cos(math.radians(lat)))
  return new_lat, new_lon


def _make_roundabout_nodes(center_lat: float, center_lon: float, radius_m: float = 20.0, n: int = 12) -> list[dict]:
  """Generate circular roundabout nodes around a center point."""
  nodes = []
  for i in range(n):
    angle = 2 * math.pi * i / n
    nlat, nlon = _offset_coordinate(center_lat, center_lon, radius_m * math.cos(angle), radius_m * math.sin(angle))
    nodes.append({'lat': nlat, 'lon': nlon})
  return nodes


class TestHelperFunctions:
  """Tests for the geometry helper functions."""

  def test_is_ahead_within_threshold(self):
    assert _is_ahead(0.0, 30.0) is True
    assert _is_ahead(0.0, 350.0) is True  # slightly left
    assert _is_ahead(90.0, 120.0) is True

  def test_is_ahead_outside_threshold(self):
    assert _is_ahead(0.0, 100.0) is False
    assert _is_ahead(0.0, 260.0) is False
    assert _is_ahead(0.0, 180.0) is False

  def test_haversine_known_distance(self):
    # ~111km per degree of latitude
    dist = _haversine_distance(55.0, 10.0, 56.0, 10.0)
    assert 110000 < dist < 112000

  def test_bearing_north(self):
    b = _bearing_to(55.0, 10.0, 56.0, 10.0)
    assert abs(b - 0.0) < 1.0 or abs(b - 360.0) < 1.0

  def test_bearing_east(self):
    b = _bearing_to(55.0, 10.0, 55.0, 11.0)
    assert abs(b - 90.0) < 1.0


class TestLateralFilterQueryOverpass:
  """Tests for lateral filtering in _query_overpass (centroid-based)."""

  def _simulate_query(self, car_lat, car_lon, bearing, elements):
    """Simulate the centroid-based filtering logic from _query_overpass."""
    min_dist = float('inf')
    nearest_lat = 0.0
    nearest_lon = 0.0
    best_center_lat = 0.0
    best_center_lon = 0.0

    for element in elements:
      nodes = element.get('geometry', [])
      if not nodes:
        continue

      center_lat = sum(n['lat'] for n in nodes) / len(nodes)
      center_lon = sum(n['lon'] for n in nodes) / len(nodes)

      if bearing is not None:
        b = _bearing_to(car_lat, car_lon, center_lat, center_lon)
        if not _is_ahead(bearing, b):
          continue
        bearing_diff_rad = math.radians(abs((b - bearing + 180) % 360 - 180))
        centroid_dist = _haversine_distance(car_lat, car_lon, center_lat, center_lon)
        lateral_offset = centroid_dist * abs(math.sin(bearing_diff_rad))
        if lateral_offset > MAX_LATERAL_OFFSET:
          continue

      for node in nodes:
        nd = _haversine_distance(car_lat, car_lon, node['lat'], node['lon'])
        if nd < min_dist:
          min_dist = nd
          nearest_lat = node['lat']
          nearest_lon = node['lon']
          best_center_lat = center_lat
          best_center_lon = center_lon

    found = min_dist < float('inf')
    return found, min_dist, nearest_lat, nearest_lon, best_center_lat, best_center_lon

  def test_parallel_road_roundabout_rejected(self):
    """Roundabout 300m ahead and 210m to the east on a parallel road -> rejected."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0  # heading north

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 300.0, 210.0)
    nodes = _make_roundabout_nodes(center_lat, center_lon)
    elements = [{'geometry': nodes}]

    found, *_ = self._simulate_query(car_lat, car_lon, bearing, elements)
    assert found is False, "Roundabout on parallel road should be rejected"

  def test_direct_ahead_roundabout_accepted(self):
    """Roundabout 220m directly ahead with only 7m lateral offset -> accepted."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 220.0, 7.0)
    nodes = _make_roundabout_nodes(center_lat, center_lon)
    elements = [{'geometry': nodes}]

    found, *_ = self._simulate_query(car_lat, car_lon, bearing, elements)
    assert found is True, "Roundabout directly ahead should be accepted"

  def test_offramp_approach_accepted(self):
    """Roundabout on an off-ramp approach (bearing ~30 degrees, close lateral) -> accepted."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 30.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 173.0, 100.0)
    nodes = _make_roundabout_nodes(center_lat, center_lon)
    elements = [{'geometry': nodes}]

    found, *_ = self._simulate_query(car_lat, car_lon, bearing, elements)
    assert found is True, "Roundabout on off-ramp approach should be accepted"

  def test_large_roundabout_near_nodes_but_offaxis_centroid_rejected(self):
    """Large roundabout with centroid 100m off-axis but near-side nodes only ~30m away -> rejected by centroid."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 150.0, 100.0)
    nodes = _make_roundabout_nodes(center_lat, center_lon, radius_m=70.0)
    elements = [{'geometry': nodes}]

    found, *_ = self._simulate_query(car_lat, car_lon, bearing, elements)
    assert found is False, "Large roundabout with off-axis centroid should be rejected even if near-side nodes are close"

  def test_roundabout_behind_rejected(self):
    """Roundabout directly behind the car -> rejected by bearing check."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, -200.0, 0.0)
    nodes = _make_roundabout_nodes(center_lat, center_lon)
    elements = [{'geometry': nodes}]

    found, *_ = self._simulate_query(car_lat, car_lon, bearing, elements)
    assert found is False, "Roundabout behind the car should be rejected"

  def test_no_bearing_skips_lateral_check(self):
    """When bearing is None (no GPS heading), skip filtering and accept all roundabouts."""
    car_lat, car_lon = 55.0, 10.0
    bearing = None

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 100.0, 300.0)
    nodes = _make_roundabout_nodes(center_lat, center_lon)
    elements = [{'geometry': nodes}]

    found, *_ = self._simulate_query(car_lat, car_lon, bearing, elements)
    assert found is True, "Without bearing, all roundabouts should be accepted (fallback)"


class TestLateralFilterUpdate:
  """Tests for lateral filtering in the update() method using cached centroid."""

  def _make_detector_with_roundabout(self, car_lat, car_lon, center_lat, center_lon,
                                     nearest_lat, nearest_lon, distance):
    """Create a detector with pre-populated roundabout state."""
    det = RoundaboutDetector()
    det._is_roundabout = True
    det._distance_to_roundabout = distance
    det._nearest_lat = nearest_lat
    det._nearest_lon = nearest_lon
    det._center_lat = center_lat
    det._center_lon = center_lon
    det._behind_count = 0
    return det

  def test_update_clears_offaxis_roundabout_after_hysteresis(self):
    """update() clears a roundabout whose centroid is off-trajectory after hysteresis."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 150.0, 80.0)
    nearest_lat, nearest_lon = _offset_coordinate(car_lat, car_lon, 150.0, 10.0)
    dist = _haversine_distance(car_lat, car_lon, nearest_lat, nearest_lon)

    det = self._make_detector_with_roundabout(car_lat, car_lon, center_lat, center_lon,
                                              nearest_lat, nearest_lon, dist)

    for _ in range(ROUNDABOUT_BEHIND_THRESHOLD):
      det.update(car_lat, car_lon, bearing)

    assert det.is_roundabout is False, "Off-axis roundabout should be cleared after hysteresis"
    assert det.distance_to_roundabout == 0.0
    assert det._center_lat == 0.0, "Centroid should be cleared after hysteresis reset"
    assert det._center_lon == 0.0, "Centroid should be cleared after hysteresis reset"
    assert det._nearest_lat == 0.0
    assert det._nearest_lon == 0.0

  def test_update_bearing_none_preserves_state(self):
    """When bearing is None, update() skips distance tracking entirely (state unchanged)."""
    car_lat, car_lon = 55.0, 10.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 200.0, 10.0)
    nearest_lat, nearest_lon = _offset_coordinate(car_lat, car_lon, 190.0, 5.0)
    dist = _haversine_distance(car_lat, car_lon, nearest_lat, nearest_lon)

    det = RoundaboutDetector()
    det._is_roundabout = True
    det._distance_to_roundabout = dist
    det._nearest_lat = nearest_lat
    det._nearest_lon = nearest_lon
    det._center_lat = center_lat
    det._center_lon = center_lon

    # bearing=None -> entire distance-update block is skipped
    det.update(car_lat, car_lon, None)

    assert det.is_roundabout is True, "State should be preserved when bearing is None"
    assert det._distance_to_roundabout == dist, "Distance should not change when bearing is None"

  def test_update_keeps_onaxis_roundabout(self):
    """update() keeps a roundabout whose centroid is on-trajectory."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0

    center_lat, center_lon = _offset_coordinate(car_lat, car_lon, 200.0, 10.0)
    nearest_lat, nearest_lon = _offset_coordinate(car_lat, car_lon, 190.0, 5.0)
    dist = _haversine_distance(car_lat, car_lon, nearest_lat, nearest_lon)

    det = self._make_detector_with_roundabout(car_lat, car_lon, center_lat, center_lon,
                                              nearest_lat, nearest_lon, dist)

    det.update(car_lat, car_lon, bearing)

    assert det.is_roundabout is True, "On-axis roundabout should be kept"
    assert det._behind_count == 0

  def test_update_no_centroid_fallback_to_bearing_only(self):
    """When centroid is not cached (0,0), update() falls back to bearing-only check."""
    car_lat, car_lon = 55.0, 10.0
    bearing = 0.0

    nearest_lat, nearest_lon = _offset_coordinate(car_lat, car_lon, 200.0, 5.0)
    dist = _haversine_distance(car_lat, car_lon, nearest_lat, nearest_lon)

    det = RoundaboutDetector()
    det._is_roundabout = True
    det._distance_to_roundabout = dist
    det._nearest_lat = nearest_lat
    det._nearest_lon = nearest_lon
    # _center_lat/_center_lon remain 0.0

    det.update(car_lat, car_lon, bearing)

    assert det.is_roundabout is True, "Without centroid, bearing-only check should accept ahead node"
    assert det._behind_count == 0


class TestConstants:
  """Verify constants are set correctly."""

  def test_max_lateral_offset(self):
    assert MAX_LATERAL_OFFSET == 60.0

  def test_early_braking_secs_reduced(self):
    assert ROUNDABOUT_EARLY_BRAKING_SECS == 7.0
