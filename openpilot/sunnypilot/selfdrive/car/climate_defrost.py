"""Offroad A/C (defroster) control for Toyota.

Why this is a UDS tool and not a CAN broadcast
----------------------------------------------
The A/C amplifier broadcasts two status messages that openpilot can already see on the
tapped buses (0x380 ACN1S01 and 0x3B0 ACN1S07, both ~1 Hz on bus 2). It does *not*
broadcast 0x381 ACN1S04, the message that carries the panel's defrost / recirculate /
blower-level switch requests, and the gateway's 0x40016303 YGW1S02_0 "YR_DEFOG" frame is
not on a tapped bus either (no extended IDs appear in the logs at all).

So there is no address on bus 0/1/2 that the A/C amplifier treats as "turn the rear
defogger on", and fabricating one would be both unverifiable and unsafe. The only route
into the amplifier is UDS diagnostics on its own address -- 0x7c4, the same ECU
openpilot already fingerprints as Ecu.hvac.

UDS requires the panda in ELM327 mode, which only permits arbitrary transmit while the
car is offroad. Every entry point here therefore refuses to run unless the vehicle is
parked, and the safety model is always restored in a finally block.

Verification is closed loop: the outcome is read back from the RDEF bit in 0x3B0, which
we can observe, so a command is only reported successful when the car actually changed.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Iterable
from typing import NamedTuple

# A/C amplifier status broadcasts (present on bus 2 via the OBD tap, src 130)
ACN1S01_ID = 0x380
ACN1S07_ID = 0x3B0

# A/C amplifier diagnostic address (Ecu.hvac in opendbc/car/toyota/values.py)
HVAC_DIAG_TX = 0x7C4
HVAC_DIAG_RX = 0x7CC
HVAC_DIAG_BUS = 0

# UDS services / parameters
SERVICE_INPUT_OUTPUT_CONTROL = 0x2F
SERVICE_ROUTINE_CONTROL = 0x31

CONTROL_RETURN_TO_ECU = 0
CONTROL_RESET_TO_DEFAULT = 1
CONTROL_FREEZE_CURRENT_STATE = 2
CONTROL_SHORT_TERM_ADJUSTMENT = 3

ROUTINE_START = 1
ROUTINE_STOP = 2

# Starting points for the probe. These are *not* known-good identifiers for this car;
# the probe exists precisely to discover which one the amplifier accepts.
CANDIDATE_IDS = (
  0x0001, 0x0002, 0x0003, 0x0100, 0x0101, 0x0102,
  0x0200, 0x0201, 0x0300, 0x0301, 0x1000, 0x1001,
)

# Tr_Temp / outside temperature scales from toyota_2017_ref_pt.dbc
TR_TEMP_SCALE, TR_TEMP_OFFSET = 0.25, -6.5
TAMOUT_SCALE = 0.625


class VehicleNotOffroadError(RuntimeError):
  """Raised when a transmit-capable operation is attempted while driving."""


class DefrostUnavailableError(RuntimeError):
  """Raised when the A/C amplifier does not answer on its diagnostic address."""


class ClimateState(NamedTuple):
  rear_defrost: bool
  mirror_heat: bool
  setpoint_c: float
  ambient_raw: int
  ac_mode: int
  ambient05: bool


class OutsideState(NamedTuple):
  outside_temp_c: float
  blower_on: bool
  rear_heater: bool
  econ: bool


def decode_acn1s07(dat: bytes) -> ClimateState:
  """ACN1S07 (0x3B0, 6 bytes) -> cabin climate status.

  Signal layout (toyota_2017_ref_pt.dbc), bits numbered from bit 0 = LSB of byte 0:
    RDEF     7|1   rear defogger active
    MHTR     6|1   mirror heater active
    TR_TEMP 15|8   cabin setpoint, 0.25 scale, -6.5 offset
    ACN_AMB 31|8   ambient raw
    AC_MODE 43|2   air outlet mode
    AC_AMB05 44|1
  """
  if len(dat) < 6:
    raise ValueError(f"ACN1S07 must be at least 6 bytes, got {len(dat)}")
  return ClimateState(
    rear_defrost=bool(dat[0] & 0x80),
    mirror_heat=bool(dat[0] & 0x40),
    setpoint_c=dat[1] * TR_TEMP_SCALE + TR_TEMP_OFFSET,
    ambient_raw=dat[3],
    # 43|2: MSB at bit 43, so the field is bits 43..42 -> byte 5 bits 3..2
    ac_mode=(dat[5] >> 2) & 0x3,
    ambient05=bool(dat[5] & 0x10),
  )


def decode_acn1s01(dat: bytes) -> OutsideState:
  """ACN1S01 (0x380, 8 bytes) -> outside temperature and heater relay flags."""
  if len(dat) < 8:
    raise ValueError(f"ACN1S01 must be 8 bytes, got {len(dat)}")
  return OutsideState(
    outside_temp_c=int.from_bytes(dat[6:7], "big", signed=True) * TAMOUT_SCALE,
    blower_on=bool(dat[1] & 0x80),   # BLWON  15|1
    rear_heater=bool(dat[0] & 0x20),  # R_HTR   5|1
    econ=bool(dat[0] & 0x04),         # ECOSW   2|1
  )


def build_input_output_control(identifier: int, control_type: int, option_record: bytes = b"") -> bytes:
  """UDS 0x2F InputOutputControlByIdentifier payload (without the ISO-TP framing)."""
  return (
    bytes([SERVICE_INPUT_OUTPUT_CONTROL])
    + identifier.to_bytes(2, "big")
    + bytes([control_type])
    + option_record
  )


def build_routine_control(routine_control_type: int, routine_id: int, record: bytes = b"") -> bytes:
  """UDS 0x31 RoutineControl payload (without the ISO-TP framing)."""
  return (
    bytes([SERVICE_ROUTINE_CONTROL, routine_control_type])
    + routine_id.to_bytes(2, "big")
    + record
  )


class elm327_session:
  """Allow arbitrary transmit while parked, then put the panda back to silent.

  The python bindings expose set_safety_mode() but no getter, so the previous mode cannot
  be read back; the contract is ELM327 on the way in and silent on the way out, which is
  the correct resting state once the stack is stopped. A failed restore is deliberately
  not swallowed -- leaving the panda in ELM327 would mean unrestricted transmit.

  Takes any object exposing set_safety_mode() so the restore path is unit testable.
  """

  def __init__(self, panda, safety_mode_elm327: int, safety_mode_silent: int = 0, safety_param: int = 1):
    self.panda = panda
    self._mode = safety_mode_elm327
    self._silent = safety_mode_silent
    self._param = safety_param

  def __enter__(self):
    self.panda.set_safety_mode(self._mode, self._param)
    return self

  def __exit__(self, exc_type, exc, tb):
    self.panda.set_safety_mode(self._silent, 0)
    return False


def safety_model(name: str) -> int:
  """Resolve a SafetyModel ordinal (elm327 / silent) from whichever cereal layout is present."""
  try:
    from openpilot.cereal import car as _car
  except ImportError:
    from cereal import car as _car
  return int(getattr(_car.CarParams.SafetyModel, name))


class DefrostController:
  """Rear defroster actuator with closed-loop verification against RDEF."""

  def __init__(self, uds, can_recv: Callable[[], Iterable[tuple[int, bytes, int]]],
               is_offroad: Callable[[], bool],
               identifier: int = 0x0001, control_type: int = CONTROL_SHORT_TERM_ADJUSTMENT,
               use_routine: bool = False, option_record: bytes = b"\x01"):
    self.uds = uds
    self.can_recv = can_recv
    self.is_offroad = is_offroad
    self.identifier = identifier
    self.control_type = control_type
    self.use_routine = use_routine
    self.option_record = option_record

  # -- state ----------------------------------------------------------------
  def read_rear_defrost(self, timeout: float = 1.5, poll: float = 0.05) -> bool | None:
    """Latest RDEF value, or None if no ACN1S07 arrived within the timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      for addr, dat, _bus in self.can_recv() or []:
        if addr == ACN1S07_ID and len(dat) >= 6:
          return decode_acn1s07(bytes(dat)).rear_defrost
      time.sleep(poll)
    return None

  # -- actuation ------------------------------------------------------------
  def build_request(self, on: bool) -> bytes:
    value = b"\x01" if on else b"\x00"
    if self.use_routine:
      return build_routine_control(ROUTINE_START, self.identifier, self.option_record if on else b"")
    return build_input_output_control(self.identifier, self.control_type, value)

  def set_rear_defrost(self, on: bool, verify_timeout: float = 4.0) -> bool:
    """Request DEFROST on/off. Returns True only if the car reports the new state."""
    if not self.is_offroad():
      raise VehicleNotOffroadError("defroster actuation is only allowed while parked")

    request = self.build_request(on)
    try:
      response = self.uds.request(request)
    except Exception as exc:  # negative response / timeout / no answer
      raise DefrostUnavailableError(
        f"A/C amplifier at {HVAC_DIAG_TX:#05x} rejected {request.hex()}: {exc}"
      ) from exc

    observed = self.read_rear_defrost(timeout=verify_timeout)
    return observed is on and response is not None


def probe(uds, identifiers: Iterable[int] = CANDIDATE_IDS, log: Callable[[str], None] = print) -> dict:
  """Ask the A/C amplifier which identifiers it accepts, without changing anything.

  Every attempt is CONTROL_FREEZE_CURRENT_STATE (2) on 0x2F or a RoutineControl request
  with an empty option record, i.e. at worst a no-op, and negative responses are recorded
  rather than raised so a full sweep completes.
  """
  results: dict[int, str] = {}
  for identifier in identifiers:
    request = build_input_output_control(identifier, CONTROL_FREEZE_CURRENT_STATE)
    try:
      uds.request(request)
      results[identifier] = "accepted(0x2F freeze)"
      log(f"  {identifier:#06x}  ACCEPTED 0x2F")
      continue
    except Exception as exc:
      results[identifier] = f"0x2F: {exc}"
    routine = build_routine_control(ROUTINE_START, identifier)
    try:
      uds.request(routine)
      results[identifier] = "accepted(0x31 start)"
      log(f"  {identifier:#06x}  ACCEPTED 0x31")
    except Exception as exc:
      results[identifier] += f" | 0x31: {exc}"
      log(f"  {identifier:#06x}  no  ({exc})")
  return results


def _make_uds(panda):
  from opendbc.car.uds import UdsClient  # imported lazily: needs no hardware to import
  return UdsClient(panda, HVAC_DIAG_TX, HVAC_DIAG_RX, bus=HVAC_DIAG_BUS)


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description="Read / actuate the A/C defroster while parked")
  group = parser.add_mutually_exclusive_group(required=True)
  group.add_argument("--state", action="store_true", help="read climate status from CAN only")
  group.add_argument("--probe", action="store_true", help="ask the A/C amplifier which UDS identifiers it accepts")
  group.add_argument("--on", action="store_true", help="turn the rear defroster on")
  group.add_argument("--off", action="store_true", help="turn the rear defroster off")
  parser.add_argument("--id", type=lambda v: int(v, 0), default=0x0001, help="UDS identifier/routine id")
  parser.add_argument("--routine", action="store_true", help="use RoutineControl (0x31) instead of 0x2F")
  parser.add_argument("--control-type", type=int, default=CONTROL_SHORT_TERM_ADJUSTMENT)
  parser.add_argument("--timeout", type=float, default=4.0)
  args = parser.parse_args(argv)

  from panda import Panda  # only needed for the hardware paths

  try:
    panda = Panda()
  except Exception as exc:
    print(f"cannot open panda ({exc}).\nThe openpilot stack owns it while running; stop it first:\n"
          f"  sudo systemctl stop comma", file=sys.stderr)
    return 2

  def can_recv():
    return panda.can_recv()

  try:
    if args.state:
      deadline = time.monotonic() + max(args.timeout, 2.0)
      seen = {}
      while time.monotonic() < deadline and len(seen) < 2:
        for addr, dat, bus in can_recv() or []:
          if addr in (ACN1S01_ID, ACN1S07_ID) and addr not in seen:
            seen[addr] = (bytes(dat), bus)
      if ACN1S07_ID in seen:
        st = decode_acn1s07(seen[ACN1S07_ID][0])
        print(f"rear_defrost={st.rear_defrost} mirror_heat={st.mirror_heat} "
              f"setpoint={st.setpoint_c:.1f}C ac_mode={st.ac_mode}")
      else:
        print("no ACN1S07 seen (car asleep?)", file=sys.stderr)
      if ACN1S01_ID in seen:
        out = decode_acn1s01(seen[ACN1S01_ID][0])
        print(f"outside={out.outside_temp_c:.2f}C blower_on={out.blower_on} "
              f"rear_heater={out.rear_heater} econ={out.econ}")
      return 0 if ACN1S07_ID in seen else 1

    if not args.on and not args.off and not args.probe:
      return 0

    with elm327_session(panda, safety_model("elm327"), safety_model("silent")):
      uds = _make_uds(panda)
      if args.probe:
        print(f"A/C amplifier {HVAC_DIAG_TX:#05x} identifier sweep:")
        results = probe(uds)
        accepted = [f"{k:#06x} -> {v}" for k, v in results.items() if v.startswith("accepted")]
        print("\naccepted: " + (", ".join(accepted) if accepted else "none"))
        return 0

      controller = DefrostController(
        uds, can_recv, is_offroad=lambda: True,  # offroad asserted by the caller below
        identifier=args.id, control_type=args.control_type, use_routine=args.routine,
      )
      want = bool(args.on)
      before = controller.read_rear_defrost()
      print(f"before: rear_defrost={before}")
      print(f"sending {controller.build_request(want).hex()} to {HVAC_DIAG_TX:#05x}")
      ok = controller.set_rear_defrost(want, verify_timeout=args.timeout)
      after = controller.read_rear_defrost()
      print(f"after:  rear_defrost={after}  -> {'OK' if ok else 'NOT CONFIRMED'}")
      return 0 if ok else 1
  finally:
    panda.close()


if __name__ == "__main__":
  raise SystemExit(main())
