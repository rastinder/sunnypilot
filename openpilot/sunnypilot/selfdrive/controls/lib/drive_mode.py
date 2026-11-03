"""
Drive-mode profiles (eco / normal / sport) for sunnypilot longitudinal control.

sunnypilot exposes one three-way longitudinal selector to the driver: Driving
Personality (relaxed / standard / aggressive). It is settable from the device UI, from
Sunnylink, and from the car's own ACC distance button, so it is the natural place to
hang a drive-mode abstraction:

  relaxed    -> eco     gentle launch, moderate gap, smoother jerk
  standard   -> normal  the default behaviour
  aggressive -> sport   sharp launch, tightest gap, sharper jerk

Two behaviours beyond the upstream personality table:

* Following time is reduced in all three modes (upstream: 1.75 / 1.45 / 1.25 s), with a
  hard safety floor so no setting can command a gap tighter than MIN_T_FOLLOW.
* ``launch_boost`` is added to the MPC acceleration limit while pulling away from a
  standstill behind a lead that has actually started moving, so the car reacts when the
  gap opens instead of waiting for the solver to converge on its own.
"""

# cereal needs capnp, which is not available in lightweight unit-test environments.
# The personalities are a plain 0/1/2 enum, so fall back to those values.
try:  # pragma: no cover - depends on the runtime environment
  from openpilot.cereal import log

  RELAXED = log.LongitudinalPersonality.relaxed
  STANDARD = log.LongitudinalPersonality.standard
  AGGRESSIVE = log.LongitudinalPersonality.aggressive
except Exception:  # pragma: no cover
  RELAXED, STANDARD, AGGRESSIVE = 0, 1, 2

# Never command a following gap tighter than this, whatever the mode says.
MIN_T_FOLLOW = 0.9

# Launch detection thresholds while stopped behind a lead.
LAUNCH_EGO_SPEED = 1.0    # m/s - ego is still (essentially) stopped below this
LAUNCH_LEAD_SPEED = 0.3   # m/s - lead counts as moving above this

# Upstream personality following times, kept so the tests can assert we stay tighter.
UPSTREAM_T_FOLLOW = {RELAXED: 1.75, STANDARD: 1.45, AGGRESSIVE: 1.25}


class DriveModeProfile:
  """Longitudinal behaviour for a single drive mode."""

  def __init__(self, name: str, t_follow: float, accel_scale: float, jerk_scale: float, launch_boost: float):
    self.name = name
    self.t_follow = max(t_follow, MIN_T_FOLLOW)
    self.accel_scale = accel_scale
    self.jerk_scale = jerk_scale
    self.launch_boost = launch_boost

  def __repr__(self) -> str:
    return (f"DriveModeProfile({self.name!r}, t_follow={self.t_follow}, "
            f"accel_scale={self.accel_scale}, jerk_scale={self.jerk_scale}, launch_boost={self.launch_boost})")


# accel_scale multiplies the vehicle's ACCEL_MAX, so 1.0 is "everything the car can do"
# (sport). Jerk scale is a cost multiplier: higher = smoother, lower = sharper.
PROFILES = {
  RELAXED: DriveModeProfile("eco", t_follow=1.40, accel_scale=0.70, jerk_scale=1.50, launch_boost=0.20),
  STANDARD: DriveModeProfile("normal", t_follow=1.10, accel_scale=0.90, jerk_scale=1.00, launch_boost=0.40),
  AGGRESSIVE: DriveModeProfile("sport", t_follow=0.90, accel_scale=1.00, jerk_scale=0.60, launch_boost=0.60),
}


def get_profile(personality) -> DriveModeProfile:
  """Profile for a personality value. Unknown values fall back to normal."""
  return PROFILES.get(personality, PROFILES[STANDARD])


def get_t_follow(personality) -> float:
  return get_profile(personality).t_follow


def get_accel_scale(personality) -> float:
  return get_profile(personality).accel_scale


def get_jerk_scale(personality) -> float:
  return get_profile(personality).jerk_scale


def get_launch_boost(personality) -> float:
  return get_profile(personality).launch_boost


def is_launching(v_ego: float, v_lead: float, lead_present: bool) -> bool:
  """True while pulling away from a stop behind a lead that has started moving."""
  return bool(lead_present and v_ego < LAUNCH_EGO_SPEED and v_lead > LAUNCH_LEAD_SPEED)
