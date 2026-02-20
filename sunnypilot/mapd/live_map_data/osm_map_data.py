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
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData
from openpilot.sunnypilot.navd.helpers import Coordinate

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
ROUNDABOUT_QUERY_RADIUS = 60  # meters
ROUNDABOUT_QUERY_INTERVAL = 5.0  # seconds between queries
ROUNDABOUT_POSITION_THRESHOLD = 30.0  # meters - only re-query if moved this far


class RoundaboutDetector:
  """Detects roundabouts using Overpass API queries in a background thread."""

  def __init__(self):
    self._is_roundabout = False
    self._last_query_lat = 0.0
    self._last_query_lon = 0.0
    self._last_query_time = 0.0
    self._lock = threading.Lock()
    self._running = True

  @property
  def is_roundabout(self) -> bool:
    with self._lock:
      return self._is_roundabout

  def update(self, lat: float, lon: float) -> None:
    """Called from main thread at 1Hz. Triggers background query if needed."""
    if lat == 0.0 and lon == 0.0:
      return

    now = time.monotonic()
    if now - self._last_query_time < ROUNDABOUT_QUERY_INTERVAL:
      return

    # Only re-query if moved significantly
    if self._last_query_lat != 0.0:
      dlat = lat - self._last_query_lat
      dlon = lon - self._last_query_lon
      # Rough distance in meters (at mid-latitudes, 1 deg lat ~ 111km, 1 deg lon ~ 65km)
      dist = math.sqrt((dlat * 111000) ** 2 + (dlon * 65000) ** 2)
      if dist < ROUNDABOUT_POSITION_THRESHOLD:
        return

    self._last_query_time = now
    thread = threading.Thread(target=self._query_overpass, args=(lat, lon), daemon=True)
    thread.start()

  def _query_overpass(self, lat: float, lon: float) -> None:
    """Background thread: query Overpass API for nearby roundabouts."""
    query = f'[out:json][timeout:3];way(around:{ROUNDABOUT_QUERY_RADIUS},{lat},{lon})[junction=roundabout];out ids;'
    try:
      data = urllib.parse.urlencode({'data': query}).encode('utf-8')
      req = urllib.request.Request(OVERPASS_URL, data=data, method='POST')
      with urllib.request.urlopen(req, timeout=3) as resp:
        result = json.loads(resp.read().decode('utf-8'))
        found = len(result.get('elements', [])) > 0

      with self._lock:
        self._is_roundabout = found
        self._last_query_lat = lat
        self._last_query_lon = lon

    except (urllib.error.URLError, json.JSONDecodeError, OSError, TimeoutError):
      # Network failure - keep previous state, don't crash
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

    # Feed position to roundabout detector
    self._roundabout_detector.update(self.last_position.latitude, self.last_position.longitude)

  def get_current_speed_limit(self) -> float:
    return float(self.mem_params.get("MapSpeedLimit") or 0.0)

  def get_current_road_name(self) -> str:
    return str(self.mem_params.get("RoadName") or "")

  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
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

  def get_advisory_speed_limit(self) -> float:
    return float(self.mem_params.get("MapAdvisorySpeedLimit") or 0.0)

  def get_is_roundabout(self) -> bool:
    return self._roundabout_detector.is_roundabout
