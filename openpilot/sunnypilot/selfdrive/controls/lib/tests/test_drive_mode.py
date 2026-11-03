import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import drive_mode


class TestDriveModeProfiles(unittest.TestCase):
  def test_three_modes_are_named_eco_normal_sport(self):
    names = set()
    for personality in (drive_mode.RELAXED, drive_mode.STANDARD, drive_mode.AGGRESSIVE):
      names.add(drive_mode.get_profile(personality).name)
    self.assertEqual(names, {"eco", "normal", "sport"})

  def test_following_time_tightens_with_mode(self):
    eco = drive_mode.get_t_follow(drive_mode.RELAXED)
    normal = drive_mode.get_t_follow(drive_mode.STANDARD)
    sport = drive_mode.get_t_follow(drive_mode.AGGRESSIVE)
    self.assertGreater(eco, normal)
    self.assertGreater(normal, sport)

  def test_following_time_never_goes_below_the_floor(self):
    for personality in (drive_mode.RELAXED, drive_mode.STANDARD, drive_mode.AGGRESSIVE):
      self.assertGreaterEqual(drive_mode.get_t_follow(personality), drive_mode.MIN_T_FOLLOW)

  def test_every_mode_is_tighter_than_the_upstream_personality_table(self):
    for personality, upstream in drive_mode.UPSTREAM_T_FOLLOW.items():
      self.assertLess(drive_mode.get_t_follow(personality), upstream)

  def test_acceleration_scales_up_with_mode(self):
    eco = drive_mode.get_accel_scale(drive_mode.RELAXED)
    normal = drive_mode.get_accel_scale(drive_mode.STANDARD)
    sport = drive_mode.get_accel_scale(drive_mode.AGGRESSIVE)
    self.assertLess(eco, normal)
    self.assertLess(normal, sport)

  def test_accel_scale_never_exceeds_vehicle_limit(self):
    for personality in (drive_mode.RELAXED, drive_mode.STANDARD, drive_mode.AGGRESSIVE):
      self.assertLessEqual(drive_mode.get_accel_scale(personality), 1.0)

  def test_eco_jerk_is_softer_than_sport(self):
    self.assertGreater(drive_mode.get_jerk_scale(drive_mode.RELAXED),
                       drive_mode.get_jerk_scale(drive_mode.AGGRESSIVE))

  def test_launch_boost_is_present_and_ordered(self):
    boosts = [drive_mode.get_launch_boost(p) for p in
              (drive_mode.RELAXED, drive_mode.STANDARD, drive_mode.AGGRESSIVE)]
    self.assertTrue(all(b > 0.0 for b in boosts))
    self.assertEqual(boosts, sorted(boosts))

  def test_unknown_personality_falls_back_to_normal(self):
    self.assertEqual(drive_mode.get_profile(99).name, "normal")


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
