import unittest

from openpilot.sunnypilot.selfdrive.car.climate_defrost import (
  ACN1S07_ID,
  CONTROL_FREEZE_CURRENT_STATE,
  CONTROL_SHORT_TERM_ADJUSTMENT,
  DefrostController,
  DefrostUnavailableError,
  ROUTINE_START,
  VehicleNotOffroadError,
  build_input_output_control,
  build_routine_control,
  decode_acn1s01,
  decode_acn1s07,
  elm327_session,
  probe,
)

# Real payloads captured from the car (bus 2, ACN1S07 has 6 meaningful bytes)
ACN1S07_IDLE = bytes.fromhex("0079003f00100000")[:6]
ACN1S07_OTHER = bytes.fromhex("0078003e00100000")[:6]
ACN1S01_14C = bytes.fromhex("0000000000001700")


class TestDecoders(unittest.TestCase):
  def test_setpoint_decodes_from_real_payload(self):
    # 0x79 = 121 -> 121 * 0.25 - 6.5 = 23.75
    self.assertAlmostEqual(decode_acn1s07(ACN1S07_IDLE).setpoint_c, 23.75, places=3)
    self.assertAlmostEqual(decode_acn1s07(ACN1S07_OTHER).setpoint_c, 23.5, places=3)

  def test_rear_defrost_bit_is_byte0_bit7(self):
    self.assertFalse(decode_acn1s07(ACN1S07_IDLE).rear_defrost)
    on = bytes([0x80]) + ACN1S07_IDLE[1:]
    self.assertTrue(decode_acn1s07(on).rear_defrost)

  def test_mirror_heat_bit_is_byte0_bit6(self):
    self.assertFalse(decode_acn1s07(ACN1S07_IDLE).mirror_heat)
    on = bytes([0x40]) + ACN1S07_IDLE[1:]
    self.assertTrue(decode_acn1s07(on).mirror_heat)

  def test_ac_mode_and_ambient05_use_the_right_bits(self):
    st = decode_acn1s07(ACN1S07_IDLE)          # byte5 = 0x10 -> amb05 set, mode 0
    self.assertEqual(st.ac_mode, 0)
    self.assertTrue(st.ambient05)
    # AC_MODE = 3 lives in byte5 bits 3..2
    self.assertEqual(decode_acn1s07(ACN1S07_IDLE[:5] + bytes([0x0C])).ac_mode, 3)
    self.assertFalse(decode_acn1s07(ACN1S07_IDLE[:5] + bytes([0x0C])).ambient05)

  def test_outside_temperature_scale(self):
    # 0x17 = 23 -> 23 * 0.625 = 14.375 C, which is the range seen across real drives
    self.assertAlmostEqual(decode_acn1s01(ACN1S01_14C).outside_temp_c, 14.375, places=3)
    self.assertAlmostEqual(decode_acn1s01(bytes.fromhex("0000000000001800")).outside_temp_c, 15.0, places=3)

  def test_short_frames_rejected(self):
    with self.assertRaises(ValueError):
      decode_acn1s07(b"\x00\x00")
    with self.assertRaises(ValueError):
      decode_acn1s01(b"\x00")


class TestRequests(unittest.TestCase):
  def test_input_output_control_bytes(self):
    self.assertEqual(build_input_output_control(0x0001, CONTROL_SHORT_TERM_ADJUSTMENT, b"\x01"),
                     bytes([0x2F, 0x00, 0x01, 0x03, 0x01]))

  def test_routine_control_bytes(self):
    self.assertEqual(build_routine_control(ROUTINE_START, 0x0201),
                     bytes([0x31, 0x01, 0x02, 0x01]))

  def test_on_off_differ(self):
    c = DefrostController(uds=None, can_recv=lambda: [], is_offroad=lambda: True)
    self.assertNotEqual(c.build_request(True), c.build_request(False))
    self.assertTrue(c.build_request(True).endswith(b"\x01"))
    self.assertTrue(c.build_request(False).endswith(b"\x00"))


class FakeUds:
  def __init__(self, response=b"\x6f", raises=None):
    self.requests = []
    self.response = response
    self.raises = raises

  def request(self, payload):
    self.requests.append(payload)
    if self.raises:
      raise self.raises
    return self.response


def can_source(rdef: bool):
  dat = bytes([0x80 if rdef else 0x00]) + ACN1S07_IDLE[1:]
  return lambda: [(ACN1S07_ID, dat, 130)]


class TestGuardAndVerification(unittest.TestCase):
  def test_refuses_when_not_offroad(self):
    uds = FakeUds()
    controller = DefrostController(uds, can_source(True), is_offroad=lambda: False)
    with self.assertRaises(VehicleNotOffroadError):
      controller.set_rear_defrost(True)
    self.assertEqual(uds.requests, [], "must not transmit while driving")

  def test_success_requires_closed_loop_confirmation(self):
    uds = FakeUds()
    ok = DefrostController(uds, can_source(True), is_offroad=lambda: True).set_rear_defrost(True, verify_timeout=0.1)
    self.assertTrue(ok)
    self.assertEqual(len(uds.requests), 1)

  def test_reports_failure_when_car_did_not_change(self):
    uds = FakeUds()
    ok = DefrostController(uds, can_source(False), is_offroad=lambda: True).set_rear_defrost(True, verify_timeout=0.1)
    self.assertFalse(ok, "a command must not be reported as successful unless RDEF flipped")

  def test_reports_failure_when_no_status_frames_arrive(self):
    uds = FakeUds()
    ok = DefrostController(uds, lambda: [], is_offroad=lambda: True).set_rear_defrost(True, verify_timeout=0.1)
    self.assertFalse(ok)

  def test_negative_response_surfaces_as_error(self):
    uds = FakeUds(raises=RuntimeError("request out of range"))
    controller = DefrostController(uds, can_source(True), is_offroad=lambda: True)
    with self.assertRaises(DefrostUnavailableError):
      controller.set_rear_defrost(True)

  def test_read_state_returns_none_without_frames(self):
    self.assertIsNone(DefrostController(FakeUds(), lambda: [], is_offroad=lambda: True).read_rear_defrost(timeout=0.1))


class FakePanda:
  SAFETY_ELM327 = 9
  SAFETY_SILENT = 0

  def __init__(self, mode=0, param=0):
    self.mode, self.param = mode, param
    self.calls = []

  def get_safety_model(self):
    return self.mode, self.param

  def set_safety_mode(self, mode, param=0):
    self.calls.append((mode, param))
    self.mode, self.param = mode, param


class TestSafetyRestore(unittest.TestCase):
  def test_restores_silent_on_success(self):
    panda = FakePanda(mode=FakePanda.SAFETY_SILENT, param=0)
    with elm327_session(panda, FakePanda.SAFETY_ELM327):
      self.assertEqual(panda.mode, FakePanda.SAFETY_ELM327)
    self.assertEqual(panda.mode, FakePanda.SAFETY_SILENT, "panda must not be left in ELM327")
    self.assertEqual(len(panda.calls), 2)

  def test_restores_silent_even_when_starting_from_a_real_safety_mode(self):
    panda = FakePanda(mode=3, param=7)
    with elm327_session(panda, FakePanda.SAFETY_ELM327):
      self.assertEqual(panda.mode, FakePanda.SAFETY_ELM327)
    self.assertEqual((panda.mode, panda.param), (FakePanda.SAFETY_SILENT, 0))

  def test_restores_silent_on_exception(self):
    panda = FakePanda(mode=3, param=7)
    with self.assertRaises(RuntimeError):
      with elm327_session(panda, FakePanda.SAFETY_ELM327):
        raise RuntimeError("boom")
    self.assertEqual(panda.mode, FakePanda.SAFETY_SILENT)


class TestProbe(unittest.TestCase):
  def test_probe_records_negative_responses_instead_of_raising(self):
    uds = FakeUds(raises=RuntimeError("sub-function not supported"))
    results = probe(uds, identifiers=(0x0001, 0x0002), log=lambda _: None)
    self.assertEqual(set(results), {0x0001, 0x0002})
    self.assertTrue(all("not supported" in v for v in results.values()))

  def test_probe_uses_freeze_not_a_real_actuation(self):
    uds = FakeUds()
    probe(uds, identifiers=(0x0001,), log=lambda _: None)
    self.assertEqual(uds.requests[0], build_input_output_control(0x0001, CONTROL_FREEZE_CURRENT_STATE))


if __name__ == "__main__":
  unittest.main()
