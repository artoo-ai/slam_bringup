# slam_bringup — Test Plan

Reusable bench + on-rig validation plan for the `slam_bringup` sensor stack. Each phase in `PLAN.md` has a matching section here; re-run the tests in a section whenever you change hardware, cabling, power, firmware, launch files, or config for that phase.

---

## How to use this document

- Test IDs are stable (`TEST-<phase>.<n>`). When you modify something, re-run the tests covering what you touched and re-tick the boxes. IDs never change even if the step list evolves.
- Each test has: **Goal**, **Hardware**, **Preconditions**, **Steps**, **Pass criteria**, **Troubleshooting**, **Results log**.
- Append a new row to the Results log every time you run a test. Never delete old rows — they are the history for "last known good".
- Checkboxes at the top of each test reflect the **current rig**. Clear them when rig hardware changes that invalidates the prior pass (new cable → clear TEST-1.6.2; new buck → clear TEST-1.6.1; etc.).

### Legend
- `[ ]` not yet passed on current rig
- `[x]` passed on current rig (see Results log for date)
- `[~]` passed with caveat — see Notes
- `[!]` currently failing / regression — do not proceed to next phase

---

## Current rig baseline (as of 2026-04-19)

Record changes here; any change invalidates the tests that reference the affected item.

| Item | Value | Notes |
|------|-------|-------|
| Compute | Jetson Orin Nano Super (`gizmo`), JetPack 6.x, Ubuntu 22.04, ROS2 Humble | |
| LiDAR | Livox Mid-360, static IP `192.168.1.202`, host `192.168.1.100` | On `eth1` via USB-GbE (Realtek r8152) |
| Front camera | Intel RealSense D435 — serial `023422073803`, FW 5.17.0.10 | USB3 (Bus 002, port 2-1.3) |
| Rear camera | Intel RealSense D435i — staged, Phase 1.10 | Not wired at this phase |
| IMU | WitMotion WT901C (USB serial, 0x61 packet format) | `/dev/ttyUSB0` @ 115200 |
| Frame | 2020 + 2040 aluminum extrusion sensor plate | — |
| Power source | Battery (record pack, nominal V, capacity Wh) → Pololu buck → 5V rail | **Record exact Pololu model + set-point here** |
| USB cables | D435: current length ___ (swap to shorter planned); WT901C: ___ | Record each swap in the Results log |
| LiDAR cable | Ethernet, length ___ | |

---

## Conventions

Unless a test says otherwise, every shell assumes:

```bash
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
```

(Already in `~/.bashrc` on `gizmo` — new shells are good to go.)

All scripts live in `~/slam_ws/src/slam_bringup/`. Start/kill scripts are idempotent: `start_sensors.sh` auto-cleans stale drivers via `kill_sensors.sh`.

Soak tests should be run **headless on the Jetson** — close rviz2, Foxglove Studio, RustDesk, Chrome, any other GUI session on the Jetson. Run rviz/Foxglove from a **remote host** (laptop over Wi-Fi) so the Jetson CPU baseline is clean. PLAN.md §6.6 documents this explicitly.

---

# Phase 1.6 — Integrated Sensor Bringup on Physical Rig

**Context:** Sensors are physically mounted on the finalized 2020/2040 frame, cabled together, and powered from battery via the Pololu buck converter. This phase proves the whole rig (not just drivers on a bench) holds nominal rates under sustained load, with no new power, thermal, mechanical, or USB regressions versus bench results.

This phase supersedes the bench-only pass of PLAN.md §6.6 (`[x]` marks there were measured on desktop-PSU bench, GUI session on Jetson). **Re-pass every test below before advancing to Phase 1.7 (URDF + measured TF offsets).**

## Checklist — Phase 1.6 exit criteria

- [ ] TEST-1.6.1 — Power integrity (battery + buck under load)
- [ ] TEST-1.6.2 — Cold-boot sensor bringup + topic rates
- [ ] TEST-1.6.3 — 15-minute steady-state soak
- [ ] TEST-1.6.4 — Thermal baseline (Jetson + buck, enclosed rig)
- [ ] TEST-1.6.5 — Mechanical rigidity + cable strain relief
- [ ] TEST-1.6.6 — Clean-session jtop baseline (updates PLAN.md §6.6)
- [ ] TEST-1.6.7 — Visualization sanity (rviz2 / Foxglove)
- [ ] TEST-1.6.8 — Short-cable regression (run after each cable swap)

---

## TEST-1.6.1 — Power Integrity (battery + buck under load)

**Status:** [ ]

**Goal:** Confirm the buck/battery path holds a steady 5V with acceptable ripple while all three sensors stream at full rate. This is the highest-risk new item — D435 frame-timeout hangs are often downstream of brown-outs, not firmware.

### Hardware needed
- Fully assembled rig (all three sensors cabled and mounted)
- Battery (fully charged — note voltage before starting)
- Pololu buck converter (record model + configured output voltage)
- Digital multimeter — **required**
- Oscilloscope — **optional** (catches transient ripple / droop the DMM will average through)
- Multimeter probes long enough to reach the 5V rail at the Jetson input without disturbing cables

### Preconditions
- Battery fully charged; log the starting voltage.
- Buck output set and verified with the multimeter **before** connecting to Jetson: should read within ±0.05 V of target (5.0 V or whatever you chose) with no load.

### Steps

1. [ ] Measure battery voltage at rest. Log it.
2. [ ] Measure buck output with no load (sensors unpowered, Jetson on wall PSU if needed to isolate). Should be target ±0.05 V. Log it.
3. [ ] Connect buck output to Jetson 5V input. Power Jetson from battery/buck only — disconnect the wall PSU.
4. [ ] Measure 5V rail at the Jetson input terminal (multimeter in DC V, probes on the header). Log idle voltage.
5. [ ] SSH in from a separate host (so the Jetson isn't running a local GUI). Run:
   ```bash
   cd ~/slam_ws/src/slam_bringup && ./start_sensors.sh
   ```
6. [ ] Watch the rs_camera log for the first 30s. Any `Frames didn't arrive within 5 seconds` → immediate fail, stop and diagnose power first.
7. [ ] With all three sensors streaming, re-measure the 5V rail. Log it.
8. [ ] If using a scope: probe the 5V rail, AC-couple, 50 mV/div, 10 ms/div. Capture a screenshot / photo and record peak-to-peak ripple.
9. [ ] In a second SSH terminal, watch for undervoltage / USB events:
   ```bash
   dmesg -wT | grep -iE 'undervolt|throttl|usb.*reset|disconn|r8152'
   ```
   Leave running for the duration of the other steps.
10. [ ] Also log Jetson-reported power draw:
    ```bash
    tegrastats --interval 1000
    ```
    Note `POM_5V_IN` (total board draw) steady-state. Log the number.
11. [ ] Ctrl-C `start_sensors.sh` cleanly, then `./kill_sensors.sh` to make sure nothing lingers.

### Pass criteria
- [ ] Buck no-load output within ±0.05 V of target
- [ ] 5V rail **under full load** ≥ 4.90 V steady (D435 tolerates down to ~4.75 V but tight margins cause frame timeouts)
- [ ] 5V rail ripple (if scoped) < 100 mVpp
- [ ] Zero D435 `Frames didn't arrive` warnings in the 30 s observation window
- [ ] Zero undervoltage / throttle / USB reset entries in `dmesg` during the run
- [ ] `tegrastats` steady-state power draw logged (for comparison on future changes)

### Troubleshooting
- **5V sags below 4.9 V under load:** buck is current-limited or input voltage too low. Check battery voltage under load, buck's rated output current vs measured draw.
- **Frame timeouts appear only when other sensors start:** shared ground noise or inrush current starving the D435. Try moving the D435 USB to a different Jetson port / separate rail.
- **`dmesg` shows `r8152` resets:** the USB-GbE adapter is power-starved or on a noisy rail — can also kill LiDAR Ethernet.

### Results log
| Date | By | Battery V start | Buck no-load V | 5V under load | Ripple pp | `POM_5V_IN` W | Result | Notes |
|------|----|----------------|----------------|---------------|-----------|---------------|--------|-------|
|      |    |                |                |               |           |               |        |       |

---

## TEST-1.6.2 — Cold-boot sensor bringup + topic rates

**Status:** [ ]

**Goal:** From a fully-off Jetson, single-command bringup produces all four critical topics at nominal rates. Covers PLAN.md §6.6's rate checks on the physical rig.

### Hardware needed
- Rig as in TEST-1.6.1
- Remote host (laptop) on the same network for running `ros2 topic hz` — keeps Jetson CPU clean

### Preconditions
- TEST-1.6.1 passed (power is known good)
- Jetson booted, no ROS processes running (`pgrep -f ros2` empty)

### Steps

1. [ ] SSH in and confirm baseline:
   ```bash
   pgrep -f ros2            # expect no output
   ros2 daemon stop         # clear any stale DDS cache
   ```
2. [ ] Start sensors:
   ```bash
   cd ~/slam_ws/src/slam_bringup && ./start_sensors.sh
   ```
3. [ ] Wait ~10 s for all drivers to initialize. Verify no red ERROR lines in the launch output (the `No valid configuration file found at /home/rico/.realsense-config.json` line is benign).
4. [ ] In a second SSH session, confirm topics exist:
   ```bash
   ros2 topic list | grep -E 'livox|d435_front|imu/data'
   ```
   Expect all five: `/livox/lidar`, `/livox/imu`, `/d435_front/camera/color/camera_info`, `/d435_front/camera/depth/camera_info`, `/imu/data`.
5. [ ] Measure rates — **one at a time**, 10 s window each:
   ```bash
   ros2 topic hz /livox/lidar               # expect ~10 Hz
   ros2 topic hz /livox/imu                 # expect ~200 Hz
   ros2 topic hz /d435_front/camera/color/camera_info   # expect ~30 Hz
   ros2 topic hz /d435_front/camera/depth/camera_info   # expect ~30 Hz
   ros2 topic hz /imu/data                  # expect ~200 Hz
   ```
   **Do NOT use `/image_raw` for rate checks** — QoS mismatch returns zero (documented gotcha in PLAN.md §6.4).
6. [ ] Confirm WitMotion self-reported rate from its own stats log (appears every 5 s in the launch output):
   ```
   [wt901c_imu] XXX.X Hz over last 5.0s (NNN packets, 0 resync bytes dropped)
   ```
   Resync bytes should be `0`. Sustained non-zero means noisy USB serial.
7. [ ] Clean shutdown: `Ctrl-C` the launch, then `./kill_sensors.sh`. Confirm no lingering processes:
   ```bash
   pgrep -af 'realsense2_camera_node|livox_ros_driver2|wt901c_imu'
   ```

### Pass criteria
- [ ] All five expected topics present
- [ ] `/livox/lidar` ≥ 9.5 Hz
- [ ] `/livox/imu` 195–205 Hz
- [ ] `/d435_front/camera/color/camera_info` ≥ 29 Hz
- [ ] `/d435_front/camera/depth/camera_info` ≥ 28 Hz
- [ ] `/imu/data` 195–205 Hz; wt901c log shows 0 resync bytes dropped
- [ ] Clean shutdown — no orphan processes

### Troubleshooting
- **Topic missing entirely:** driver didn't start. Check launch log for that driver's section.
- **`/d435_front/...` rates at 0 and log shows frame timeouts:** D435 firmware hang. See `feedback_d435_frames_timeout.md`. Run `./kill_sensors.sh` (triggers the sysfs deauth/reauth in `kill_d435.sh`) and re-launch.
- **`/livox/*` rates at 0:** Mid-360 network not reachable. `ping 192.168.1.202`; check `host_net_info` IPs in `config/mid360.json` (must be `192.168.1.100`, not `255.255.255.255` — see memory `feedback_mid360_host_ip.md`).
- **`/imu/data` rate low:** WT901C packet-type mismatch or serial collision with another device on `/dev/ttyUSB0`. Check `dmesg | grep ttyUSB`.

### Results log
| Date | By | `/livox/lidar` Hz | `/livox/imu` Hz | color_info Hz | depth_info Hz | `/imu/data` Hz | wt901c resync | Result | Notes |
|------|----|-------------------|-----------------|---------------|---------------|----------------|---------------|--------|-------|
|      |    |                   |                 |               |               |                |               |        |       |

---

## TEST-1.6.3 — 15-minute steady-state soak

**Status:** [ ]

**Goal:** Rates that pass a 10 s measurement window can still drift, stall, or crash at 10 minutes. This test catches thermal throttling, memory leaks, USB-power drift, and buck ripple accumulation.

### Hardware needed
- Rig as in TEST-1.6.1
- Kitchen timer / phone timer
- Remote SSH session (no Jetson GUI)

### Preconditions
- TEST-1.6.2 passed
- Battery charge level noted at start (for discharge curve)

### Steps

1. [ ] Record battery voltage at start. Log it.
2. [ ] In SSH session 1: `cd ~/slam_ws/src/slam_bringup && ./start_sensors.sh` — keep the launch log visible.
3. [ ] In SSH session 2: start a long `dmesg` watch:
    ```bash
    dmesg -wT | tee ~/soak_dmesg.log
    ```
4. [ ] In SSH session 3: `tegrastats --interval 5000 > ~/soak_tegrastats.log`
5. [ ] Start a 15-minute timer.
6. [ ] At T+5 min, T+10 min, T+15 min: re-measure the four main topic rates (short 10s window each) and log:
    ```bash
    ros2 topic hz /livox/lidar
    ros2 topic hz /imu/data
    ros2 topic hz /d435_front/camera/color/camera_info
    ros2 topic hz /d435_front/camera/depth/camera_info
    ```
7. [ ] At T+15 min, record:
    - Battery voltage (log it)
    - Multimeter reading on 5V rail (for comparison with TEST-1.6.1)
    - Max Jetson CPU temp from `tegrastats` (grep `CPU@`, look for highest)
8. [ ] Stop `tegrastats` (Ctrl-C session 3), stop `dmesg` watch (Ctrl-C session 2).
9. [ ] Ctrl-C the launch. `./kill_sensors.sh`.
10. [ ] Grep the soak logs:
    ```bash
    grep -iE 'undervolt|throttl|usb.*reset|disconn|r8152|drop' ~/soak_dmesg.log
    grep -iE 'frames didn' <wherever you tee'd the launch output>
    ```
    (Tip: redirect the launch itself with `./start_sensors.sh 2>&1 | tee ~/soak_launch.log`.)

### Pass criteria
- [ ] All four rates within ±10% of TEST-1.6.2 values at every checkpoint (5, 10, 15 min)
- [ ] Zero D435 frame-timeout warnings in the launch log
- [ ] Zero undervoltage / USB reset entries in dmesg
- [ ] Jetson max CPU temp < 80 °C
- [ ] Battery voltage drop over 15 min recorded (for future discharge-curve comparison)
- [ ] Clean shutdown, no orphan processes

### Troubleshooting
- **Rates degrade only at T+10+ min:** thermal throttling (cross-check with TEST-1.6.4) or buck heating causing droop (check buck temperature by touch — should be warm, not hot).
- **Sudden drop to 0 Hz on one topic:** driver crashed or USB disconnect. Grep `~/soak_dmesg.log` for that time window.
- **Battery sag toward end of soak brings D435 timeouts:** buck headroom too tight. Either increase buck set-point (e.g. 5.1 V) or fix battery capacity.

### Results log
| Date | By | Batt V start | Batt V end | Rates stable? | Max CPU °C | dmesg clean? | Result | Notes |
|------|----|--------------|------------|---------------|------------|--------------|--------|-------|
|      |    |              |            |               |            |              |        |       |

---

## TEST-1.6.4 — Thermal baseline (Jetson + buck, enclosed rig)

**Status:** [ ]

**Goal:** Document steady-state and worst-case temperatures for all heat sources on the mounted rig. A bench setup with open airflow does not predict enclosed-rig temps.

### Hardware needed
- IR thermometer **or** IR camera **or** contact thermocouple — any one will work
- Rig in its mounted/enclosed configuration (final mechanical setup, not lifted off the frame)

### Preconditions
- TEST-1.6.1 passed
- Ambient room temp recorded

### Steps

1. [ ] Ambient temp — record.
2. [ ] With rig off and cold: measure surface temp of Jetson SoC heatsink, buck converter, D435 body, Mid-360 body. These are your "cold baseline" — log them.
3. [ ] Start sensors (`./start_sensors.sh`). Begin 15-minute timer.
4. [ ] At T+5, T+10, T+15 min: measure surface temp of the four components. Also record `tegrastats` `CPU@` reading at each timestamp.
5. [ ] Look at the heat source order: buck → D435 → Mid-360 → Jetson is typical hottest→coolest under pure-sensor load.
6. [ ] Clean shutdown.

### Pass criteria
- [ ] Jetson SoC (per `tegrastats` `CPU@`) ≤ 80 °C at T+15 min
- [ ] Buck converter case temp ≤ 70 °C (depends on model — check datasheet; record it so you notice trends)
- [ ] D435 body ≤ 50 °C (Intel spec's max skin temp for D435 is 35 °C ambient at normal use; closely monitor if it runs hotter)
- [ ] Mid-360 body ≤ 50 °C (Livox spec: -20 to 55 °C operating)
- [ ] No sensor reported an over-temp warning in its logs

### Troubleshooting
- **Buck running hot:** undersized for the current draw, or enclosed with no airflow. Pololu step-downs spec a max efficiency point; verify you're not past it.
- **Jetson throttling:** check Jetson cooling (fan on? heatsink clean? thermal pad seated?).
- **D435 hot:** USB bus over-currenting, or mount is thermally coupling it to the buck.

### Results log
| Date | By | Ambient °C | Jetson CPU °C | Buck °C | D435 °C | Mid-360 °C | Result | Notes |
|------|----|------------|---------------|---------|---------|------------|--------|-------|
|      |    |            |               |         |         |            |        |       |

---

## TEST-1.6.5 — Mechanical rigidity + cable strain relief

**Status:** [ ]

**Goal:** Prove the frame is rigid enough that Mid-360 and D435 extrinsics won't drift, and that tugging on any cable does not disturb a sensor.

### Hardware needed
- Remote host running Foxglove (or rviz) for live visualization
- The rig, sitting on a stable surface

### Preconditions
- TEST-1.6.2 passed
- `start_bench_tf.sh` running so Mid-360 + D435 clouds can be co-displayed (until Phase 1.7 URDF lands)

### Steps

1. [ ] On Jetson: `./start_sensors.sh`. On Jetson separately: `./start_bench_tf.sh`. Optionally: `./start_foxglove.sh`.
2. [ ] On remote host: open Foxglove. Display frame `livox_frame`. Add `/livox/lidar` and `/d435_front/camera/depth/color/points` as PointCloud2 displays. Confirm both clouds render and roughly align.
3. [ ] **Rigidity check — tap test:** With everything streaming, lightly tap the frame near each sensor with a finger (not a hammer). Watch the combined point cloud: a rigid frame makes both clouds rattle together as a single object. If the D435 cloud moves independently of the Mid-360 cloud, the plate is flexing.
4. [ ] **Rigidity check — twist test:** Gently try to twist the frame (small torque by hand, don't go ballistic). Mid-360 cloud and D435 cloud should still track as one.
5. [ ] **Cable strain check:** For each cable (D435 USB, Mid-360 Ethernet, WT901C USB, battery/buck lead), gently tug at the connector. In the other SSH window:
   ```bash
   dmesg -wT | grep -iE 'usb|disconn|r8152'
   ```
   No entries should appear. Topic rates should not dip.
6. [ ] **Pose stability:** Put the rig on a flat surface, do nothing for 30 s. Take a Foxglove screenshot of the accumulated D435 point cloud. Pick the rig up, rotate 180°, replace. Compare: geometry should be stable, no drift relative to `livox_frame`.

### Pass criteria
- [ ] Tap test: clouds move together (no relative flex)
- [ ] Twist test: clouds move together
- [ ] Cable tug test: no USB/disconnect events, no rate drops
- [ ] Strain relief: no single cable pull can disconnect a sensor during normal handling
- [ ] No cable is under tension at rest (loops / service slack present)

### Troubleshooting
- **Clouds flex independently:** the sensor plate is not rigid. Add cross-bracing / thicker plate.
- **Cable tug causes disconnect:** strain relief missing. Add clip, P-clamp, or cable-tie anchor near the connector.

### Results log
| Date | By | Tap OK? | Twist OK? | Cable tug OK? | Result | Notes |
|------|----|---------|-----------|---------------|--------|-------|
|      |    |         |           |               |        |       |

---

## TEST-1.6.6 — Clean-session jtop baseline (updates PLAN.md §6.6)

**Status:** [ ]

**Goal:** Fill in the pending PLAN.md §6.6 row: "`jtop` baseline → CPU ~30%, RAM ~500 MB (pending — needs measurement on a clean Jetson with no GUI session)". Values here become the reference point for spotting future regressions (e.g. when FAST-LIO2 is added in Phase 2).

### Hardware needed
- Rig in normal operating configuration
- Remote laptop for the ROS commands

### Preconditions
- Jetson is **headless**: no local rviz2, Foxglove Studio, RustDesk, Chrome, or Code — ideally log out the desktop session entirely. SSH only.
- `jtop` installed (`sudo pip3 install jetson-stats` if not).

### Steps

1. [ ] Confirm the Jetson is truly headless:
    ```bash
    pgrep -af 'gnome|Xorg|rustdesk|rviz|foxglove|chrome|code'   # expect empty
    ```
2. [ ] Start jtop in its own SSH session: `jtop`. Go to the **5 ALL** tab.
3. [ ] Log baseline **idle** (sensors not running): CPU %, RAM MB, Power W, Temp °C.
4. [ ] In another SSH session: `./start_sensors.sh`.
5. [ ] Wait 3 minutes for steady state. Then from jtop, log: CPU %, RAM MB, Power W, Temp °C.
6. [ ] `Ctrl-C` launch, `./kill_sensors.sh`.
7. [ ] Update PLAN.md §6.6 line 813 with the measured values (replace the "*pending*" note).

### Pass criteria
- [ ] Steady-state CPU ≤ 40% (PLAN.md predicts ~30%)
- [ ] Steady-state RAM ≤ 700 MB (PLAN.md predicts ~500 MB)
- [ ] Values recorded and written back into PLAN.md §6.6

### Results log
| Date | By | Idle CPU % | Idle RAM MB | Load CPU % | Load RAM MB | Load Power W | Result | Notes |
|------|----|------------|-------------|------------|-------------|--------------|--------|-------|
|      |    |            |             |            |             |              |        |       |

---

## TEST-1.6.7 — Visualization sanity (rviz2 / Foxglove)

**Status:** [ ]

**Goal:** Prove an operator can see all three sensor streams co-rendered on a remote viewer — this is the bar you need to pass for Phase 1.7 URDF measurement and for demos.

### Hardware needed
- Remote host (laptop) with rviz2 **or** Foxglove Studio installed and on same network as Jetson

### Preconditions
- TEST-1.6.2 passed
- `start_foxglove.sh` works (run locally on Jetson) or Cyclone DDS discovery works cross-host

### Steps

1. [ ] Jetson: `./start_sensors.sh`
2. [ ] Jetson: `./start_bench_tf.sh` (publishes `livox_frame → camera_link` static TF — the pre-URDF stopgap)
3. [ ] Jetson: `./start_foxglove.sh` (if using Foxglove over WebSocket) — else skip
4. [ ] On remote host, open viewer and connect (Foxglove: `ws://gizmo:8765` — or rviz2 with the same DDS domain)
5. [ ] Add displays:
    - PointCloud2: `/livox/lidar`
    - PointCloud2: `/d435_front/camera/depth/color/points`
    - Image: `/d435_front/camera/color/image_raw`
    - IMU: `/imu/data`
6. [ ] Fixed frame: `livox_frame`
7. [ ] Walk around the rig and confirm: live Mid-360 cloud updates, live D435 color image, D435 pointcloud visible and roughly aligned with LiDAR on flat surfaces, IMU arrow rotates when rig is tilted.

### Pass criteria
- [ ] All four displays live
- [ ] No "Fixed Frame unknown" / "TF lookup failed" red text
- [ ] Mid-360 + D435 clouds visually co-align on a flat wall within a few cm (rough — Phase 1.7 URDF will tighten this)
- [ ] IMU orientation matches physical tilt

### Troubleshooting
- **"No transform from `d435_front_link` to `livox_frame`":** `start_bench_tf.sh` not running.
- **Foxglove doesn't connect:** `pgrep -f foxglove_bridge` — check the bridge is actually up. Firewall blocking port 8765?
- **Clouds align poorly:** expected at this phase — TF is a rough hand-measured estimate. Phase 1.7 fixes this.

### Results log
| Date | By | All 4 displays live? | Clouds align? | Result | Notes |
|------|----|---------------------|---------------|--------|-------|
|      |    |                     |               |        |       |

---

## TEST-1.6.8 — Short-cable regression

**Status:** [ ]

**Goal:** When you swap in the shorter D435 USB cable and the shorter LiDAR Ethernet cable, re-prove TEST-1.6.1 through TEST-1.6.3. Shorter cables should *improve* headroom; any regression indicates a bad replacement cable.

### Hardware needed
- New shorter D435 USB3 cable (record length + manufacturer)
- New shorter LiDAR Ethernet cable (record length)

### Preconditions
- TEST-1.6.1, TEST-1.6.2, TEST-1.6.3 currently passing on the old cables

### Steps

1. [ ] Record old cable lengths in the rig baseline table (if not already).
2. [ ] Swap D435 cable. **Physically unplug old, plug new.** Don't hot-swap — fully power the rig down first.
3. [ ] Re-run TEST-1.6.2. If it passes, re-run TEST-1.6.3.
4. [ ] Update rig baseline table with new D435 cable spec.
5. [ ] Repeat steps 2–4 for the LiDAR Ethernet cable.
6. [ ] After both swaps: final re-run of TEST-1.6.1 (power integrity — shorter cables may shift the ripple picture slightly).

### Pass criteria
- [ ] Rates after cable swaps are equal to or better than before
- [ ] No new `Frames didn't arrive` events after D435 cable swap
- [ ] No new `r8152` resets after LiDAR cable swap
- [ ] Power integrity (TEST-1.6.1) still passing

### Troubleshooting
- **Frame timeouts return after D435 cable swap:** replacement cable is USB2-only despite the label, or out-of-spec. Swap back to the old cable to isolate.
- **LiDAR drops out:** Cat5 vs Cat5e/6 matters less than connector quality — try a different cable or re-seat the RJ45.

### Results log
| Date | By | D435 old len | D435 new len | LiDAR old len | LiDAR new len | Result | Notes |
|------|----|--------------|--------------|---------------|---------------|--------|-------|
|      |    |              |              |               |               |        |       |

---

## Phase 1.6 sign-off

When all TEST-1.6.* boxes are `[x]`:

- [ ] Update the Phase 1.6 checklist at the top of this section
- [ ] Update PLAN.md §6.6 checkboxes to match
- [ ] Commit `TEST_PLAN.md` with the filled-in Results logs
- [ ] You are cleared to start Phase 1.7 (URDF + measured TF offsets)

---

# Phase 1.7 — URDF + measured TF offsets (placeholder)

*To be written when Phase 1.7 starts. Will include: measuring sensor offsets with calipers on the rig, writing the URDF per-platform, verifying `tf2_tools view_frames` produces a fully-connected tree, and cross-checking extrinsics with a known target (e.g. a corner of a room shows up in both Mid-360 and D435 at the same place).*

---

# Phase 1.8 — Platform SDK / base_link integration (placeholder)

*To be written when a specific robot platform (Go2 / R2D2 / Roboscout / mecanum) is targeted.*

---

# Phase 1.10 — Dual camera (front + rear D435i) (placeholder)

*To be written when rear D435i is wired. Will re-run power integrity, soak, and thermal tests under the higher load. Verify `lsusb -t` shows each camera on its own USB 3 controller — can't share bandwidth.*

---

# Phase 2 — SLAM layer (placeholder)

*FAST-LIO2 + RTABMap. Initial test `start_fast_lio.sh` already on disk. Full test section to be written when Phase 2 work begins: odometry stability, loop closure, octomap quality, RTABMap database persistence.*
