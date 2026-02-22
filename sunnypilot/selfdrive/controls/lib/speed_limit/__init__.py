"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
LIMIT_ADAPT_ACC = -1.  # m/s^2 Ideal acceleration for the adapting (braking) phase when approaching speed limits.
LIMIT_MAX_MAP_DATA_AGE = 10.  # s Maximum time to hold to map data, then consider it invalid inside limits controllers.

# Speed Limit Assist constants
PCM_LONG_REQUIRED_MAX_SET_SPEED = {
  True: (33.3333, 36.1111),  # km/h, (120, 130)
  False: (31.2928, 35.7632),  # mph, (70, 80)
}

CONFIRM_SPEED_THRESHOLD = {
  True: 30,   # km/h
  False: 20,  # mph
}

SPEED_INCREASE_LOOKAHEAD_TIME = 5.0  # seconds before zone to start speed increase
MAX_SPEED_INCREASE_LOOKAHEAD = 250.  # meters, cap for sensor glitch safety

SUDDEN_LIMIT_DEBOUNCE_TIME = 3.0  # seconds to hold a sudden (not seen ahead) speed limit before accepting it

ICBM_RESPONSE_BUFFER = 4.0  # seconds — extra braking lookahead for ICBM button-press latency + vehicle response
