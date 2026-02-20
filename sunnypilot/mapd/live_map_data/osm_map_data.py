"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import platform
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData
from openpilot.sunnypilot.navd.helpers import Coordinate

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
ROUNDABOUT_QUERY_RADIUS = 500  # meters - large enough for braking lookahead at highway speeds
ROUNDABOUT_QUERY_INTERVAL = 3.0  # seconds between queries
ROUNDABOUT_POSITION_THRESHOLD = 15.0  # meters - only re-query if moved this far

# Roundabout braking constants (SUVAT kinematics)
ROUNDABOUT_DECEL = -1.2  # m/s² - comfortable but firm deceleration
ROUNDABOUT_BUFFER = 50.0  # meters - arrive at target speed this far before the roundabout
ROUNDABOUT_TARGET_SPEED = 30 * CV.KPH_TO_MS  # 30 km/h target for roundabouts


def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Calculate distance in meters between two lat/lon points using haversine formula."""
  R = 6371000.0  # Earth radius in meters
  dlat = math.radians(lat2 - lat1)
  dlon = math.radians(lon2 - lon1)
  a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
  return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Calculate bearing in degrees (0-360) from point 1 to point 2."""
  dlon = math.radians(lon2 - lon1)
  lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
  x = math.sin(dlon) * math.cos(lat2_r)
  y = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
  return math.degrees(math.atan2(x, y)) % 360


def _is_ahead(car_bearing: float, bearing_to_point: float, threshold: float = 90.0) -> bool:
  """Check if a point is roughly ahead of the car (within threshold degrees of heading)."""
  diff = (bearing_to_point - car_bearing + 180) % 360 - 180
  return abs(diff) < threshold


class RoundaboutDetector:
  """Detects roundabouts ahead using Overpass API queries in a background thread.

  Uses bearing to only report roundabouts that are ahead of the car.
  Recalculates distance every update cycle using cached node position
  for smooth, real-time distance tracking between API queries.
  """

  def __init__(self):
    self._is_roundabout = False
    self._distance_to_roundabout = 0.0
    self._nearest_lat = 0.0  # cached nearest-ahead node for real-time distance
    self._nearest_lon = 0.0
    self._last_query_lat = 0.0
    self._last_query_lon = 0.0
    self._last_query_time = 0.0
    self._lock = threading.Lock()

  @property
  def is_roundabout(self) -> bool:
    with self._lock:
      return self._is_roundabout

  @property
  def distance_to_roundabout(self) -> float:
    with self._lock:
      return self._distance_to_roundabout

  def update(self, lat: float, lon: float, bearing: float) -> None:
    """Called from main thread at 1Hz. Updates distance and triggers queries."""
    if lat == 0.0 and lon == 0.0:
      return

    # Real-time distance update using cached nearest-ahead node
    with self._lock:
      if self._nearest_lat != 0.0 and bearing != 0.0:
        bearing_to_node = _bearing_to(lat, lon, self._nearest_lat, self._nearest_lon)
        if _is_ahead(bearing, bearing_to_node):
          self._distance_to_roundabout = _haversine_distance(lat, lon, self._nearest_lat, self._nearest_lon)
        else:
          # Roundabout is now behind us - clear
          self._is_roundabout = False
          self._distance_to_roundabout = 0.0
          self._nearest_lat = 0.0
          self._nearest_lon = 0.0

    # Trigger background query if enough time/distance has passed
    now = time.monotonic()
    if now - self._last_query_time < ROUNDABOUT_QUERY_INTERVAL:
      return

    if self._last_query_lat != 0.0:
      dlat = lat - self._last_query_lat
      dlon = lon - self._last_query_lon
      dist = math.sqrt((dlat * 111000) ** 2 + (dlon * 65000) ** 2)
      if dist < ROUNDABOUT_POSITION_THRESHOLD:
        return

    self._last_query_time = now
    thread = threading.Thread(target=self._query_overpass, args=(lat, lon, bearing), daemon=True)
    thread.start()

  def _query_overpass(self, lat: float, lon: float, bearing: float) -> None:
    """Background thread: query Overpass API for nearby roundabouts ahead."""
    query = f'[out:json][timeout:3];way(around:{ROUNDABOUT_QUERY_RADIUS},{lat},{lon})[junction=roundabout];out geom;'
    try:
      data = urllib.parse.urlencode({'data': query}).encode('utf-8')
      req = urllib.request.Request(OVERPASS_URL, data=data, method='POST')
      with urllib.request.urlopen(req, timeout=3) as resp:
        result = json.loads(resp.read().decode('utf-8'))
        elements = result.get('elements', [])

        min_dist = float('inf')
        nearest_lat = 0.0
        nearest_lon = 0.0

        for element in elements:
          for node in element.get('geometry', []):
            # Only consider nodes that are ahead of the car
            if bearing != 0.0:
              b = _bearing_to(lat, lon, node['lat'], node['lon'])
              if not _is_ahead(bearing, b):
                continue
            d = _haversine_distance(lat, lon, node['lat'], node['lon'])
            if d < min_dist:
              min_dist = d
              nearest_lat = node['lat']
              nearest_lon = node['lon']

        found = min_dist < float('inf')

      with self._lock:
        self._is_roundabout = found
        self._distance_to_roundabout = min_dist if found else 0.0
        self._nearest_lat = nearest_lat
        self._nearest_lon = nearest_lon
        self._last_query_lat = lat
        self._last_query_lon = lon

    except (urllib.error.URLError, json.JSONDecodeError, OSError, TimeoutError):
      cloudlog.debug("roundabout detector: overpass query failed")


class OsmMapData(BaseMapData):
  def __init__(self):
    super().__init__()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self._roundabout_detector = RoundaboutDetector()

  def update_location(self) -> None:
    location = self.sm['liveLocationKalman']
    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    if self.localizer_valid:
      self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])
      self.last_position = Coordinate(location.positionGeodetic.value[0], location.positionGeodetic.value[1])

    if self.last_position is None:
      return

    params = {
      "latitude": self.last_position.latitude,
      "longitude": self.last_position.longitude,
    }

    if self.last_bearing is not None:
      params['bearing'] = self.last_bearing

    self.mem_params.put("LastGPSPosition", json.dumps(params))

    # Feed position and bearing to roundabout detector
    self._roundabout_detector.update(self.last_position.latitude, self.last_position.longitude, self.last_bearing or 0.0)

  def get_current_speed_limit(self) -> float:
    return float(self.mem_params.get("MapSpeedLimit") or 0.0)

  def get_current_road_name(self) -> str:
    return str(self.mem_params.get("RoadName") or "")

  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    # Roundabout advisory: report approaching roundabout as "speed limit ahead"
    # This feeds into SpeedLimitResolver's existing lookahead braking logic
    if self._roundabout_detector.is_roundabout:
      dist = self._roundabout_detector.distance_to_roundabout
      if dist > 0:
        return ROUNDABOUT_TARGET_SPEED, dist

    # Normal next speed limit from map data
    next_speed_limit_section_str = self.mem_params.get("NextMapSpeedLimit")
    next_speed_limit_section = next_speed_limit_section_str if next_speed_limit_section_str else {}
    next_speed_limit = next_speed_limit_section.get('speedlimit', 0.0)
    next_speed_limit_latitude = next_speed_limit_section.get('latitude')
    next_speed_limit_longitude = next_speed_limit_section.get('longitude')
    next_speed_limit_distance = 0.0

    if next_speed_limit_latitude and next_speed_limit_longitude:
      next_speed_limit_coordinates = Coordinate(next_speed_limit_latitude, next_speed_limit_longitude)
      next_speed_limit_distance = (self.last_position or Coordinate(0, 0)).distance_to(next_speed_limit_coordinates)

    return next_speed_limit, next_speed_limit_distance
