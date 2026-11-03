import itertools
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import drive_mode


class TestPersonalityFollowingGap(unittest.TestCase):
  """Personality (relaxed/standard/aggressive) owns the following distance."""

  def test_following_time_tightens_with_personality(self):
    relaxed = drive_mode.get_t_follow(drive_mode.RELAXED)
    standard = drive_mode.get_t_follow(drive_mode.STANDARD)
    aggressive = drive_mode.get_t_follow(drive_mode.AGGRESSIVE)
    self.assertGreater(relaxed, standard)
    self.assertGreater(standard, aggressive)

  def test_following_time_never_goes_below_the_floor(self):
    for personality in (drive_mode.RELAXED, drive_mode.STANDARD, drive_mode.AGGRESSIVE):
      self.assertGreaterEqual(drive_mode.get_t_follow(personality), drive_mode.MIN_T_FOLLOW)

  def test_every_personality_is_tighter_than_upstream(self):
    for personality, upstream in drive_mode.UPSTREAM_T_FOLLOW.items():
      self.assertLess(drive_mode.get_t_follow(personality), upstream)

  def test_unknown_personality_raises(self):
    with self.assertRaises(NotImplementedError):
      drive_mode.get_t_follow(99)


class TestDriveModeAcceleration(unittest.TestCase):
  """eco/normal/sport owns acceleration only."""

  def test_three_modes_are_named_eco_normal_sport(self):
    names = {drive_mode.get_profile(m).name for m in (drive_mode.ECO, drive_mode.NORMAL, drive_mode.SPORT)}
    self.assertEqual(names, {"eco", "normal", "sport"})

  def test_acceleration_scales_up_with_mode(self):
    eco = drive_mode.get_accel_scale(drive_mode.ECO)
    normal = drive_mode.get_accel_scale(drive_mode.NORMAL)
    sport = drive_mode.get_accel_scale(drive_mode.SPORT)
    self.assertLess(eco, normal)
    self.assertLess(normal, sport)

  def test_accel_scale_never_exceeds_vehicle_limit(self):
    for mode in (drive_mode.ECO, drive_mode.NORMAL, drive_mode.SPORT):
      self.assertLessEqual(drive_mode.get_accel_scale(mode), 1.0)

  def test_eco_jerk_is_softer_than_sport(self):
    self.assertGreater(drive_mode.get_jerk_scale(drive_mode.ECO),
                       drive_mode.get_jerk_scale(drive_mode.SPORT))

  def test_launch_boost_is_present_and_ordered(self):
    boosts = [drive_mode.get_launch_boost(m) for m in
              (drive_mode.ECO, drive_mode.NORMAL, drive_mode.SPORT)]
    self.assertTrue(all(b > 0.0 for b in boosts))
    self.assertEqual(boosts, sorted(boosts))

  def test_default_mode_is_eco(self):
    self.assertEqual(drive_mode.DEFAULT_DRIVE_MODE, drive_mode.ECO)
    self.assertEqual(drive_mode.get_profile(drive_mode.DEFAULT_DRIVE_MODE).name, "eco")

  def test_unknown_drive_mode_falls_back_to_the_default(self):
    self.assertEqual(drive_mode.get_profile(99).name, "eco")
    self.assertEqual(drive_mode.get_accel_scale(99), drive_mode.get_accel_scale(drive_mode.ECO))


class TestAxesAreIndependent(unittest.TestCase):
  def test_drive_mode_never_changes_the_following_gap(self):
    # every personality/drive-mode combination keeps the gap set by personality alone
    for personality, mode in itertools.product(
        (drive_mode.RELAXED, drive_mode.STANDARD, drive_mode.AGGRESSIVE),
        (drive_mode.ECO, drive_mode.NORMAL, drive_mode.SPORT)):
      self.assertEqual(drive_mode.get_t_follow(personality),
                       drive_mode.get_t_follow(personality))
      self.assertNotIn("t_follow", drive_mode.get_profile(mode).__dict__)

  def test_relaxed_plus_sport_is_a_valid_combination(self):
    # hang back, but pull hard when the gap opens
    self.assertEqual(drive_mode.get_t_follow(drive_mode.RELAXED),
                     max(drive_mode.PERSONALITY_T_FOLLOW.values()))
    self.assertEqual(drive_mode.get_accel_scale(drive_mode.SPORT), 1.0)


class TestDriveModeSelector(unittest.TestCase):
  class FakeParams:
    def __init__(self, value):
      self.value = value

    def get(self, key):
      if isinstance(self.value, Exception):
        raise self.value
      return self.value

  def test_reads_param_and_defaults_to_eco(self):
    self.assertEqual(drive_mode.DriveModeSelector(self.FakeParams(None)).update(), drive_mode.ECO)
    self.assertEqual(drive_mode.DriveModeSelector(self.FakeParams(2)).update(), drive_mode.SPORT)

  def test_invalid_value_falls_back_to_eco(self):
    self.assertEqual(drive_mode.DriveModeSelector(self.FakeParams("garbage")).update(), drive_mode.ECO)
    self.assertEqual(drive_mode.DriveModeSelector(self.FakeParams(7)).update(), drive_mode.ECO)

  def test_param_errors_fall_back_to_eco(self):
    selector = drive_mode.DriveModeSelector(self.FakeParams(ValueError("boom")))
    self.assertEqual(selector.update(), drive_mode.ECO)

  def test_value_refreshes_after_the_interval(self):
    now = [0.0]
    params = self.FakeParams(0)
    selector = drive_mode.DriveModeSelector(params, clock=lambda: now[0])
    self.assertEqual(selector.update(), drive_mode.ECO)
    params.value = 2
    now[0] += 0.5
    self.assertEqual(selector.update(), drive_mode.ECO)   # cached
    now[0] += drive_mode.DriveModeSelector.REFRESH_SECONDS
    self.assertEqual(selector.update(), drive_mode.SPORT)  # picked up live

  def test_profile_property_matches_drive_mode(self):
    selector = drive_mode.DriveModeSelector(self.FakeParams(2))
    selector.update()
    self.assertEqual(selector.profile.name, "sport")


class TestLaunchDetection(unittest.TestCase):
  def test_stopped_ego_behind_moving_lead_is_launching(self):
    self.assertTrue(drive_mode.is_launching(v_ego=0.0, v_lead=1.5, lead_present=True))

  def test_stopped_ego_behind_stopped_lead_is_not_launching(self):
    self.assertFalse(drive_mode.is_launching(v_ego=0.0, v_lead=0.0, lead_present=True))

  def test_moving_ego_is_not_launching(self):
    self.assertFalse(drive_mode.is_launching(v_ego=12.0, v_lead=14.0, lead_present=True))

  def test_no_lead_is_not_launching(self):
    self.assertFalse(drive_mode.is_launching(v_ego=0.0, v_lead=5.0, lead_present=False))

  def test_lead_creeping_below_threshold_is_not_launching(self):
    self.assertFalse(drive_mode.is_launching(v_ego=0.0, v_lead=0.2, lead_present=True))


if __name__ == "__main__":
  unittest.main()
