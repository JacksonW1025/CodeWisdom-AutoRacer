# Acceptance Tools

These tools support phase acceptance without moving the vehicle by default.

Run from the `CodeWisdom-AutoRacer/` repository root.

## Phase 1 Offline Checks

```bash
python3 tools/acceptance/stage1_acceptance_check.py --mode offline
```

Covered cases:

- `protocol_downlink`: Ackermann v1 11-byte command frame, flags, BCC, units, clamps.
- `telemetry_uplink`: 24-byte telemetry parsing, status flags/bits, BCC rejection.
- `wheel_odom_gate`: `counts_per_meter` and steering-valid gates before `/wheel_odom`.
- `launch_contract`: `ackermann_chassis.launch.py` and EKF `/odom` contract.

## Phase 1 Live Topic Check

Start the relevant ROS nodes first. This command only inspects topics and types.

```bash
python3 tools/acceptance/stage1_acceptance_check.py --mode live
python3 tools/acceptance/stage1_acceptance_check.py --mode live --require-odom
```

Use `--require-odom` only after `/imu/data`, `/wheel_odom`, and EKF `/odom` are expected to be running.

## Phase 2 Fixture Check

```bash
python3 tools/acceptance/stage2_path_fixture_check.py
```

The fixtures under `tools/acceptance/fixtures/stage2_paths/` are tracker inputs for
the next stage. They are not a tracker implementation and do not publish commands.
