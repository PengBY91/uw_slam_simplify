# `adapters/holoocean` — HoloOcean sensor gateway (recording path)

Real replacement for the legacy `ocean_t/` scripts (see the platform architecture's section 22.3 remediation list and `uw_holoocean_adapter/__init__.py`'s docstring for the specific audited bugs this fixes). Produces the same canonical MCAP/protobuf bags (`schemas/proto/uw/domain/`) that `apps/tools/synth_bag_gen` and `apps/replay_demo` use on the C++ side — a bag written here is directly readable by `replay_demo` without translation.

Scope note: this package now contains only the **offline recording path** (simulator → canonical MCAP bag → C++ replay). The realtime closed-loop line (realtime ROS session, fault injection, task scoring, gates, scripted pilot, ArduSub SITL bridge) lived here historically and was removed from main; the full snapshot is preserved on the `archive/rov-realtime-line2` branch.

## What's real vs. not tested here

This machine has no HoloOcean/Unreal install, so `holoocean_driver.py`'s `HoloOceanSession` (the part that calls into HoloOcean itself) is written but not exercised here. Everything else IS tested (`pytest`):

- `coordinates.py` — UE↔body coordinate transforms, quaternion-based (fixes the Euler gimbal-lock branch found in `ocean_t`'s `CoordTransformer._SE3_to_pose`).
- `canonical_writer.py` — MCAP writer/reader matching `runtime/include/uw/runtime/mcap_io.hpp`'s wire format exactly (uncompressed, so bags stay cross-language readable with the C++ side's zstd/lz4-disabled MCAP build).
- `scenario_randomization.py` — the programmable multi-axis randomization API that replaces `water_control_panel.py`'s two-slider GUI panel.
- `time_utils.py` — capture/receive time separation.
- `record_session.py` — a "keyframe" (stereo pair + the GT/depth evidence keyed off it) forms only on ticks where both cameras publish, but sonar/IMU/DVL (and even a lone/monocular camera) are written on ANY tick where HoloOcean published them, independent of the camera pair or of each other — matching real hardware running each sensor at its own rate. Each of those independently-written sensors' `observation_id` is keyed on its own raw simulation-tick index, never on the (much lower-rate) camera keyframe counter — see `tests/test_record_session.py`.
- `calibrate_camera.py` — chessboard stereo calibration from a recorded bag → `configs/rig/*.yaml` (see `docs/calibration/` at repo root for mounting constraints and acceptance).

## Tools

Scripts under `tools/` are not part of the installed package; run them with `.venv/bin/python`.

- `tools/sonar_stats.py` — sonar image statistics (Rayleigh noise floor, signal-to-clutter, range attenuation, per-beam profile) for one bag or an A/B comparison; procedure in `docs/sonar-stats-procedure.md`, tested by `tests/test_sonar_stats.py` on synthetic frames + MCAP.
- `../../tools/calib/imu_allan.py` — Allan-deviation IMU noise identification -> rig `imu_noise` YAML; lives in the repo-level `tools/calib/` but runs under this venv (see `tools/calib/README.md`), tested by `tests/test_imu_allan.py`.

`--plot` on the stats tool needs the `plot` extra (`uv pip install -e ".[plot]"`).

## Setup

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python -e ".[dev]"
../../tools/codegen/gen_py.sh     # generates schema_pb2/ from schemas/proto/ — not checked in
.venv/bin/python -m pytest tests/
```
