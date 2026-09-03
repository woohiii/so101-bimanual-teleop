# SO-101 Bimanual Teleop

Bimanual (2 leader + 2 follower) SO-101 teleoperation with a live depth camera view,
2 wrist camera views, and a motor-safety clamp. Built on top of
[LeRobot](https://github.com/huggingface/lerobot) — no core LeRobot code was changed,
this repo is just the glue: launcher scripts, config, and this runbook.

## Hardware

- 2x SO-101 leader arm, 2x SO-101 follower arm, all pre-calibrated, ids
  `leader_left` / `leader_right` / `follower_left` / `follower_right`
  (`arms.json`, vendored in this repo — see "Files in this repo").
- 1x Orbbec Astra S depth camera (OpenNI2/`primesense`).
- 2x USB wrist cameras, identified by USB product name: "Innomaker", "USB 2.0 PC Cam"
  (`cameras.json`, vendored in this repo).

## Files in this repo

Everything needed to run teleop lives here — `arms.json`, `cameras.json`,
`camera_utils.py` (camera-index resolution + the published-depth-file reader) and
`camera_preview.py` were copied in from `~/ROBOTICS_PROJECT` (also on GitHub, public,
at [woohiii/-1](https://github.com/woohiii/-1)) because they're small, pure-Python,
and have no binary dependency.

**Not vendored, on purpose:** `run_astra_depth_watchdog.sh` and `astra_s_depth_hub.py`
stay in `woohiii/-1` — they need the Orbbec OpenNI2 SDK, a large vendored binary
redistribution tree living next to them there, and duplicating that into a second repo
just to move two small scripts isn't worth it (YAGNI). This repo links to them by path
instead of copying them.

**Known duplication, accepted for now:** `arms.json`/`cameras.json` now exist in both
repos. If you recalibrate an arm or a camera re-enumerates, update `woohiii/-1`'s copy
(that's what `calibrate_arms.py` writes to) and re-copy it here — there's no sync
automation, on purpose, for a 2-file config at this stage.

## Why Python

SO-101 uses Feetech smart servos — the motor firmware does its own position PID, the
host only needs to hit ~60 Hz over serial. That's not a hard-real-time workload that
would justify C++/Rust. LeRobot's entire hardware stack (motor bus, calibration,
bimanual robot/teleoperator classes) and this rig's calibration/camera tooling are
already Python — switching languages would mean re-implementing all of that for no
control-loop benefit. Staying in Python is the fast path to an MVP.

## Why 3 separate processes (not cameras-in-teleop)

It's tempting to attach the depth camera + wrist cameras to `robot.cameras` and let
`lerobot-teleoperate` show everything in one `rerun` window. On this rig, don't:
the Astra S is only reachable through OpenNI2, and running OpenNI2 and OpenCV
`VideoCapture` in the *same process* has been confirmed (via `py-spy`) to starve
OpenNI2's USB event thread and hang its native reads forever. This Astra S unit is
also known to wedge on its own even standalone, which is why a watchdog with a
`usbreset` recovery exists. So the proven layout is 3 independent processes:

| # | venv | what | why |
|---|------|------|-----|
| 1 | `lerobot`'s `.venv` (uv) | this repo's `run_bimanual_teleop.sh` → `lerobot-teleoperate` | drives all 4 arms only, no cameras attached |
| 2 | `~/lerobot_song_venv` (headless) | `woohiii/-1`'s `run_astra_depth_watchdog.sh` | owns the Astra S, publishes depth to `/tmp/vsp_astra_depth_mm.npy`, auto-recovers via USB reset |
| 3 | `~/lerobot_song_venv` (GUI opencv) | this repo's `camera_preview.py` | reads the published depth file + opens both wrist cams directly, shows 3 live `cv2.imshow` windows |

(`lerobot`'s uv venv ships headless opencv — `cv2.imshow` doesn't work there at all,
which is another reason camera display has to live in `lerobot_song_venv`.)

## Cameras inside the Rerun window (optional)

`run_bimanual_teleop_with_cameras.py` gets the wrist cams + depth into the *same*
Rerun window as the arm telemetry, without breaking the constraint above:

- **Wrist cams** go through the normal `robot.cameras` path (plain `OpenCVCameraConfig`
  per arm) — that's the same mechanism `lerobot-teleoperate` already supports via CLI
  flags (see its own module docstring for a bimanual example). No OpenNI2 involved, so
  no conflict running it in the teleop process.
- **Astra depth** still never touches OpenNI2 in this process. The script reads the
  array `run_astra_depth_watchdog.sh` already publishes to
  `/tmp/vsp_astra_depth_mm.npy` (the exact same file `camera_preview.py` reads for its
  own depth panel) and merges it into the observation dict before logging — Rerun's
  own image-key logic turns any `HxWx1` array into a `DepthImage` panel automatically.

The Astra watchdog (process #2 above) is still required — this script only *reads*
what it publishes, it doesn't replace it. `camera_preview.py` (process #3) becomes
optional at that point; keep it running too if you still want the plain `cv2.imshow`
windows alongside Rerun.

## Motor safety

The one setting that actually prevents motor damage here is `max_relative_target`
(a built-in LeRobot follower-config field, off by default). It caps how far the
follower's goal position may move per control step relative to its *current* position
— see `ensure_safe_goal_position` in LeRobot's `robots/utils.py`. Without it, if the
leader and follower start out at different positions (e.g. right after connecting),
the follower snaps straight to the leader's position on the very first command —
the classic way to grind a gearbox or stall a servo on real hardware.

`run_bimanual_teleop.sh` sets `max_relative_target=10.0` (degrees, since these configs
default to `use_degrees=True`) on both follower arms — started at `5.0` (LeRobot's own
docs use that for the Reachy2 arm) and raised to `10.0` after real teleop felt
noticeably laggy on fast moves: `5.0` capped joint speed at 60 Hz to ~300°/s, so a
quick leader motion made the follower visibly "catch up" instead of tracking live.
`10.0` (~600°/s cap) removed most of that perceived lag while still bounding a single
step, so a stale reading or a big leader/follower gap still can't cause an instant
full-range jump — just a faster-than-`5.0`, still-bounded one. If it's still too
laggy, raise it further, but each increase trades some of that jump protection away;
if it now overshoots/oscillates, come back down.
The gripper already gets hardware overload/current-limit register writes from
LeRobot's own `so_follower.configure()` — nothing to add there.

**Known gaps, left out on purpose for this MVP:** no software watchdog/kill-switch on
the SO-101 control loop itself (LeRobot only has that for the `lekiwi` mobile base),
and no live temperature/current polling during teleop. Ctrl+C is already safe —
`disconnect(disable_torque=True)` is the default, so torque is released on exit.
Add a watchdog/e-stop as a follow-up if unattended runs ever need it.

## Running it

Check ports first — USB re-enumeration can shuffle `/dev/ttyACM*`:

```bash
ls -l /dev/ttyACM*
cat ~/so101-bimanual-teleop/arms.json
```

Then, in 2 terminals:

```bash
# 1) Astra S depth watchdog (own process, own venv) - always required, lives in woohiii/-1
~/ROBOTICS_PROJECT/calibration/run_astra_depth_watchdog.sh

# 2) bimanual teleop + cameras inside the same Rerun window
~/so101-bimanual-teleop/run_bimanual_teleop_with_cameras.sh
```

Or, for separate `cv2.imshow` camera windows instead of Rerun-embedded ones, swap
step 2 for the plain arms-only launcher plus `camera_preview.py` in its own terminal:

```bash
# 2) bimanual teleop, arms only
~/so101-bimanual-teleop/run_bimanual_teleop.sh

# 3) camera preview: depth + 2 wrist windows
~/lerobot_song_venv/bin/python ~/so101-bimanual-teleop/camera_preview.py
```

Stop with **Ctrl+C** in each terminal (not Ctrl+Z / closing the terminal) — that's what
runs the cleanup that releases torque and the camera/serial handles. Both launcher
scripts also auto-clean a stopped leftover from a previous sloppy exit before starting,
but a clean Ctrl+C is still the better habit (see Troubleshooting).

## Troubleshooting

- **`Failed to open OpenCVCamera(4)` / `Could not connect on port '...'`:** almost
  always a previous run left stopped (Ctrl+Z'd, or its terminal was closed) instead of
  Ctrl+C'd, still holding the camera or serial port. Both launcher scripts now resume +
  interrupt any such leftover before starting, so this should self-heal — if it doesn't,
  `ps aux | grep run_bimanual` and `fuser /dev/video4 /dev/ttyACM*` to find and
  `kill -CONT <pid>; kill -INT <pid>` it manually.
- **`FileNotFoundError: ... /dev/ttyACMn`:** the port genuinely changed (USB
  re-enumeration, not a code bug). Run `ls -l /dev/serial/by-id/` to match each adapter's
  stable serial to its current `/dev/ttyACMn`, update `arms.json` (this repo's copy AND
  `woohiii/-1`'s), and retry.

## Verification

1. **Static checks (motors idle):**
   ```bash
   ~/lerobot_song_venv/bin/python ~/so101-bimanual-teleop/camera_preview.py --self-test
   uv run --project ~/lerobot python ~/ROBOTICS_PROJECT/calibration/calibrate_arms.py --verify
   ```
   Expect all wrist cams to resolve and all 4 arms to report `PASS`.
2. **Short dry run:** `run_bimanual_teleop.sh --teleop_time_s=15` — should hold ~60 Hz in
   the console cadence report and exit cleanly, no `ConnectionError`/`RuntimeError`.
3. **Safety clamp firing:** move a leader arm quickly while it differs from its
   follower's position — the console should print `Relative goal position magnitude
   had to be clamped to be safe.` This confirms the clamp is actually active, not just
   configured.
4. **Cameras:** either the 3 `camera_preview.py` windows ("Astra S Depth", "Wrist 1",
   "Wrist 2") update live, or — with `run_bimanual_teleop_with_cameras.sh` — the Rerun
   window shows `left_wrist` / `right_wrist` / `astra_depth` panels alongside the
   joint-position time series, all updating live.
5. **Full teleop:** with all 3 processes running, move each leader joint (including
   both grippers) and confirm the matching follower joint tracks it.
