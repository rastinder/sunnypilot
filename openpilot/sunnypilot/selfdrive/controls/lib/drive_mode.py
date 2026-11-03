"""
Two independent longitudinal axes for sunnypilot.

1. Driving Personality (relaxed / standard / aggressive)
   How close the car follows and how eagerly it closes a gap. This is the existing
   sunnypilot selector (device UI, Sunnylink, and the car's own ACC distance button) and
   keeps its meaning: personality decides the following time.

2. Drive mode (eco / normal / sport)
   How hard the car accelerates once it decides to. Selected with the DriveModeProfile
   param (0 = eco, 1 = normal, 2 = sport) and read live so it can be changed while
   driving. Drive mode does NOT change the following gap.
   Eco is the default (FrogPilot ships Sport as its default acceleration profile; this
   fork deliberately starts gentle and lets you opt into normal/sport).

They are deliberately separate: relaxed + sport (hang back, but pull hard when the gap
opens) and aggressive + eco (sit close, accelerate gently) are both valid combinations.

Following times are reduced from upstream (1.75 / 1.45 / 1.25 s) but keep their relative
order, with a hard safety floor so no setting can command a tighter gap than
MIN_T_FOLLOW.
"""

import time

# cereal needs capnp, which is not available in lightweight unit-test environments.
# Both selectors are plain 0/1/2 enums, so fall back to those values.
try:  # pragma: no cover - depends on the runtime environment
  from openpilot.cereal import log

  RELAXED = log.LongitudinalPersonality.relaxed
  STANDARD = log.LongitudinalPersonality.standard
  AGGRESSIVE = log.LongitudinalPersonality.aggressive
except Exception:  # pragma: no cover
  RELAXED, STANDARD, AGGRESSIVE = 0, 1, 2

ECO, NORMAL, SPORT = 0, 1, 2

# Drive mode is selected through this param so it can change without a rebuild.
DRIVE_MODE_PARAM = "DriveModeProfile"

# Used when the param is unset, unreadable or unrecognised: start gentle.
DEFAULT_DRIVE_MODE = ECO

# Never command a following gap tighter than this, whatever the personality says.
MIN_T_FOLLOW = 0.9

# Personality (following) table: upstream is 1.75 / 1.45 / 1.25 s.
PERSONALITY_T_FOLLOW = {
  RELAXED: 1.40,
  STANDARD: 1.10,
  AGGRESSIVE: 0.90,
}
UPSTREAM_T_FOLLOW = {
  RELAXED: 1.75,
  STANDARD: 1.45,
  AGGRESSIVE: 1.25,
}

# Launch detection thresholds while stopped behind a lead.
LAUNCH_EGO_SPEED = 1.0    # m/s - ego is still (essentially) stopped below this
LAUNCH_LEAD_SPEED = 0.3   # m/s - lead counts as moving above this


class DriveModeProfile:
  """Acceleration behaviour for one drive mode. Following distance is not here."""

  def __init__(self, name: str, accel_scale: float, jerk_scale: float, launch_boost: float):
    self.name = name
    self.accel_scale = accel_scale
    self.jerk_scale = jerk_scale
    self.launch_boost = launch_boost

  def __repr__(self) -> str:
    return (f"DriveModeProfile({self.name!r}, accel_scale={self.accel_scale}, "
            f"jerk_scale={self.jerk_scale}, launch_boost={self.launch_boost})")


# accel_scale multiplies the vehicle's ACCEL_MAX, so 1.0 is everything the car can do
# (sport). jerk_scale is a cost multiplier: higher = smoother, lower = sharper.
DRIVE_MODES = {
  ECO: DriveModeProfile("eco", accel_scale=0.70, jerk_scale=1.50, launch_boost=0.20),
  NORMAL: DriveModeProfile("normal", accel_scale=0.90, jerk_scale=1.00, launch_boost=0.40),
  SPORT: DriveModeProfile("sport", accel_scale=1.00, jerk_scale=0.60, launch_boost=0.60),
}


def _check_personality(personality) -> None:
  if personality not in (RELAXED, STANDARD, AGGRESSIVE):
    raise NotImplementedError("Longitudinal personality not supported")


def get_t_follow(personality) -> float:
  """Following time for a driving personality. This is the gap axis."""
  _check_personality(personality)
  return max(PERSONALITY_T_FOLLOW[personality], MIN_T_FOLLOW)


def get_profile(drive_mode) -> DriveModeProfile:
  """Acceleration profile for a drive mode. Unknown values fall back to the default."""
  return DRIVE_MODES.get(drive_mode, DRIVE_MODES[DEFAULT_DRIVE_MODE])


def get_accel_scale(drive_mode) -> float:
  return get_profile(drive_mode).accel_scale


def get_jerk_scale(drive_mode) -> float:
  return get_profile(drive_mode).jerk_scale


def get_launch_boost(drive_mode) -> float:
  return get_profile(drive_mode).launch_boost


def is_launching(v_ego: float, v_lead: float, lead_present: bool) -> bool:
  """True while pulling away from a stop behind a lead that has started moving."""
  return bool(lead_present and v_ego < LAUNCH_EGO_SPEED and v_lead > LAUNCH_LEAD_SPEED)


class DriveModeSelector:
  """Reads the DriveModeProfile param, refreshing periodically so changes apply live."""

  REFRESH_SECONDS = 2.0

  def __init__(self, params=None, clock=time.monotonic):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self._params = params
    self._clock = clock
    self._last_read = float("-inf")
    self._drive_mode = DEFAULT_DRIVE_MODE

  @property
  def drive_mode(self) -> int:
    return self._drive_mode

  def update(self) -> int:
    now = self._clock()
    if now - self._last_read >= self.REFRESH_SECONDS:
      self._last_read = now
      try:
        raw = self._params.get(DRIVE_MODE_PARAM)
        self._drive_mode = int(raw) if raw not in (None, "") else DEFAULT_DRIVE_MODE
      except Exception:
        self._drive_mode = DEFAULT_DRIVE_MODE
      if self._drive_mode not in DRIVE_MODES:
        self._drive_mode = DEFAULT_DRIVE_MODE
    return self._drive_mode

  @property
  def profile(self) -> DriveModeProfile:
    return get_profile(self._drive_mode)
