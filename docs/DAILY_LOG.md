# Daily Engineering Log

Issues, investigations, and resolutions on the SLAM rover project — the
running story of what broke, how we figured it out, and what fixed it.

**Conventions:** newest day first. Each issue gets: *Symptom →
Investigation → Root cause → Fix → Status.* Reference commits by hash.
Open threads end each day's entry so the next session knows where to
pick up. Update this file as problems arise, not after the fact.

---

## 2026-06-11 — First trustworthy tilt reading: ~7°

- Jetson synced; the final (expected-height) `measure_lidar_tilt.py` ran
  clean: height **0.323 m vs URDF 0.329 — consistent**, RMS 8.2 mm,
  **roll +4.57°, pitch +5.29°, total 6.99°**.
- Implication: at 7°, floor returns leak into the 0.15–0.45 m obstacle
  band beyond ≈ **1.2 m** — the floor-leak hypothesis from yesterday is
  partially REHABILITATED. The phantom speckle/ring had two real causes:
  rotation smear (fixed by the 0.6 rad/s cap) AND floor leak from mount
  tilt (fix pending). Likely physical cause: off-center fixture on
  unevenly compressed rubber isolators.
- Note: the script reads the raw lidar topic, so it reports physical tilt
  regardless of URDF — post-fix verification is map quality, not a rerun
  of the script.

### Open threads

- [ ] Confirm the 7° is mount tilt, not parking-spot tilt: flat open
      floor, roll to a second spot + heading, rerun — agree within ~0.5°
      → paste `livox_rpy 0.079793 0.092401 0.0`, `./build.sh`.
      (If the plate itself is tilted, the rotation arguably belongs on
      the `base_link_to_sensor_plate` joint so the IMU/D435 inherit it —
      fine to defer until Option B uses the plate IMU.)
- [ ] Fresh explore after the rpy lands: speckle beyond ~1.2 m should be
      gone; spins no longer smear (0.6 cap). If clean, field-test resume.
- Carried from 2026-06-10: exact `robot_radius` measurement, Option B
  decision. Repo sync + tilt rerun are DONE.

---

## 2026-06-10 — The exploration debugging marathon

Context: `start_explore_2d.sh` (2D stack: Mid-360 → pointcloud_to_laserscan
→ rf2o + slam_toolbox + Nav2 + explore_lite) could not complete an
exploration run. Started with two reported issues; live debugging across
four runs uncovered five distinct root causes stacked on top of each other.

### Issue 1 — Robot clips corners/edges (fixture off-center)

- **Symptom:** rover catches on corners and door frames while exploring.
- **Root cause:** `robot_radius: 0.18` covered the bare chassis bounding
  circle only; the sensor fixture rides off-center and overhangs it. (A
  later finding — phantom obstacles from map smear, Issue 5 — also
  contributed to wall trouble.)
- **Fix:** `robot_radius` 0.18 → 0.25 and `inflation_radius` 0.20 → 0.30
  in both costmaps (`config/nav2_params_2d.yaml`). Commit `a56e826`.
  Refine later by measuring base_link → farthest fixture corner; the
  precise alternative is an asymmetric `footprint:` polygon.
- **Status:** deployed, in effect after the mid-day rebuild.

### Issue 2 — Resume from saved map never localizes (sometimes 180° off)

- **Symptom:** resuming exploration from a serialized map placed the
  robot wrong, spun without translating, drew bogus loop-closure lines.
- **Root cause:** resume hardwired `map_start_at_dock: True` — seeds the
  pose at the graph's FIRST node (position AND heading). The Karto scan
  matcher only corrects ~±0.25 m / ±20° from the seed
  (`correlation_search_space_dimension: 0.5`,
  `coarse_search_angle_offset: 0.349`). slam_toolbox has no global
  relocalization in mapping mode, so a 180° placement error is
  mathematically unrecoverable and corrupts the graph.
- **Fix (commits `a56e826`, `519c565`):**
  - New `map_start_pose:=x,y,θ` launch arg (slam_2d.launch.py).
  - `explore_manager` continuously writes `<map>.pose` (robot's live
    map-frame pose, 2 s cadence + shutdown snapshot) so the resting pose
    survives the session.
  - `start_explore_2d.sh resume:=true` auto-seeds from the `.pose` file;
    `start_at_dock:=true` forces dock anchoring when the robot was
    carried back. Seed priority: explicit pose > start_at_dock > .pose
    file > dock fallback. Documented in README "Autonomous exploration".
- **Status:** deployed; first resume after this change still needs a dock
  start (old maps have no `.pose` file yet). Not yet field-tested.

### Issue 3 — Exploration stalls at startup: planner failures + spin loops

- **Symptom:** fresh runs barely progressed; robot spun in place (red
  odom-arrow fans in RViz at 2-3 spots), `/explore/status` showed
  frontier_count climbing while free_cells crawled.
- **Investigation:** first run also had a stale config (`build.sh` not
  run — old 0.18 footprint). After rebuild, behavior improved but still
  stalled. Relaunched RViz with `rviz_config:=nav_2d.rviz` to see
  costmaps; logs showed `GridBased: failed to create plan with tolerance
  0.50` — GLOBAL PLANNER failures, not controller failures. Costmap
  showed a speckle ring of phantom obstacles surrounding the robot,
  inflation closing all free space.
- **Wrong hypothesis (worth remembering):** the ring looked like floor
  returns leaking into the 0.15–0.45 m scan slice from a tilted lidar
  mount. Wrote `scripts/measure_lidar_tilt.py` (commit `ddd10dd`) to
  floor-fit the tilt — which led to Issues 5 and 6 instead. The ring
  turned out to be Issue 4: walls smeared through untracked rotation,
  painted at every intermediate angle = a ring centered on the spin spot.
- **Status:** superseded by Issues 4–6. The planner failures were a
  downstream symptom of phantom obstacles from map smear.

### Issue 4 — In-place rotation smears the map (THE core failure)

- **Symptom:** "It rotated and the map got smeared" — rotational
  ghost-walls after every spin recovery.
- **Investigation:** caught live in the logs: `yahboom_bridge: cmd_vel →
  wz=+1.000` while rf2o reported yaw moving at **0.03 rad/s** — the
  laser odometry missed ~97% of a physical rotation.
- **Root cause:** `/scan` is a 100 ms pointcloud_to_laserscan aggregation
  of the Mid-360's non-repetitive sweep. At 1.0+ rad/s each "scan" is
  internally smeared 6–9°; pure rotation is rf2o's worst case anyway.
  slam_toolbox then gets rotated scans against a stationary odom seed,
  overruns its ±20° window, commits misregistered nodes — unrecoverable.
  This fed a vicious cycle: smear → phantom obstacles → planner failures
  (Issue 3) → recovery spins → more smear.
- **Fix:** cap ALL rotation speed at **0.6 rad/s** (≈3.4°/scan,
  trackable): DWB `max_vel_theta`, behavior_server spin, velocity_smoother
  theta. `acc_lim_theta` stays 5.0 — mecanum stiction is broken by torque
  impulse, not top speed. Commit `773487f`. Memory + config comments
  updated so this never gets "re-optimized" back up.
- **Durable fix (open):** Option B — wheel encoders + IMU EKF. A gyro
  trivially tracks what laser odom can't. The 0.6 cap is a hard ceiling
  for the laser-only stack.
- **Status:** committed; first post-cap field run pending.

### Issue 5 — `/scan` stalls for seconds; CPU starvation on the Orin

- **Symptom:** stretches of `rf2o: Waiting for laser_scans....` with no
  odom output for 2+ s; `Extrapolation Error` in explore_lite; `Message
  Filter dropping message`; `BT tick rate 100.00 exceeded`; spin recovery
  `Exceeded time allowance`.
- **Root cause(s):** CPU starvation. RViz was running ON the Jetson
  (`rviz:=true`) alongside Foxglove; jtop showed `Jetson Clocks:
  inactive` (cores floating 729 MHz–1.3 GHz) plus a full desktop session
  (gnome-shell 30% + Xorg 15% + warp-terminal 18% + RustDesk).
- **Fix/practice:** no RViz on the Jetson (Foxglove only) — confirmed
  gaps dropped from 2+ s to occasional ~0.4 s. Before each session:
  `sudo jetson_clocks`. Consider headless boot
  (`sudo systemctl set-default multi-user.target`) for real runs.
- **Status:** practice established; rf2o exec still spikes to ~70 ms vs
  100 ms budget — keep an eye on it.

### Issue 6 — URDF lidar height was wrong by 18.4 cm

- **Symptom:** `measure_lidar_tilt.py` reported the lidar 0.081 m above
  the floor; URDF stack-up claimed 0.144 m.
- **Investigation:** tape measure — plate top is **11.5 in = 0.29210 m**
  off the ground. The URDF's 0.107670 was a placeholder (its own comment
  said so). Both numbers were wrong; the script's 0.081 was a furniture
  plane (see Issue 7).
- **Fix:** `base_link_to_plate_top` 0.107670 → 0.29210
  (mecanum.urdf.xacro) and `PLATFORM_BRIDGES['mecanum']` z −0.144280 →
  −0.328710 (slam.launch.py), kept in lockstep. Lidar optical center:
  0.32871 m. Commit `4258ef4`.
- **Why it mattered:** with the wrong height, the 0.15–0.45 m obstacle
  slice actually sampled 0.33–0.63 m above the real floor — knee-height
  obstacles were invisible to the costmap.
- **Status:** committed; needs rebuild on the Jetson.

### Issue 7 — Tilt-measurement script: four selection strategies to get
### the floor right

Finding "the floor" in a Mid-360 cloud from a 0.33 m mount turned out to
be genuinely adversarial. Field runs + synthetic tests killed three
strategies in sequence:

1. **Densest z-bin** — rejected in synthetic testing before deployment:
   a *tilted* floor spreads across many 2 cm bins while a chair seat
   concentrates in two, so density picks the distractor.
2. **Most RANSAC inliers** (commit `ddd10dd`) — field run reported
   "level, height 0.081 m"; tape measure refuted the height. From 0.33 m
   the −7.2° FOV edge only reaches the floor beyond ~2.6 m at grazing
   angle, so the floor is SPARSE and a couch/bed out-votes it.
3. **Lowest well-supported plane** (commit `4258ef4`) — two field runs
   returned "floors" 2.38 m and 0.99 m BELOW the sensor, tilted 41° and
   20°: **mirror/glass reflection ghosts** form coherent planes below the
   real floor. The URDF height cross-check (added same commit) caught
   both — "OFF BY +1.891 m" — so no bad rpy was ever applied.
4. **Final: expected-height anchor.** The tape-measured mount height is
   the one trustworthy reference. Selection = best-supported near-level
   (≤8°) plane within ±0.05 m of −0.32871; the window is deliberately
   tight because at ±0.15 a mixed plane slicing couch+floor obliquely won
   at the window edge (synthetic). Post-refinement re-validation errors
   out if the fit drifts from the window instead of reporting garbage.
   Validated synthetically: height to the mm and tilt to ±0.02° against
   couch + sub-floor mirror ghost + walls simultaneously.

- **Status:** final version committed on the Mac — **sync to the Jetson
  and rerun.** Trust a result ONLY if the height line says "consistent".
  Several evening reruns on the robot used stale copies mid-iteration;
  check for the "expected-height" wording in the output to confirm the
  right version is running.

### Lessons of the day

1. **Measure before fixing** — the tilt hypothesis was persuasive and
   wrong; the tape measure killed it in 10 seconds.
2. A sanity cross-check (measured height vs URDF) turns a silently-wrong
   measurement into a loud one. Build that into tooling from the start.
3. Slow-moving config (URDF placeholders, un-rebuilt workspaces, stale
   scripts on the robot) caused three separate false trails today. Verify
   *deployed* values (`ros2 param get …`), not repo values.
4. On mecanum + laser-only odometry, rotation speed is a SLAM parameter,
   not just a motion parameter.

### Open threads for next session

- [ ] Sync repo to Jetson (`git pull` + `./build.sh`) — rotation cap,
      URDF height, footprint, resume seeding are all committed but the
      robot ran stale copies all evening.
- [ ] Rerun `measure_lidar_tilt.py` (final expected-height version);
      apply `livox_rpy` only if the height line says "consistent". If it
      errors about clutter, point the robot at a few meters of open floor
      and use `-p duration:=30.0`.
- [ ] First clean post-fix exploration run: `sudo jetson_clocks`, no
      RViz, fresh map. Watch for smear during the (now slower) spins.
- [ ] Field-test resume seeding (`.pose` auto-seed + `start_at_dock`).
- [ ] Measure base_link → farthest fixture corner; set exact
      `robot_radius`.
- [ ] Decide on Option B (wheel odom + IMU EKF) — the durable fix for
      rotation tracking.

---

## Earlier context (pre-log)

- `2670e7e` / `2191c8d` — corner-wedging first addressed via inflation
  bump (0.15 → 0.20) and explore-script teardown hardening (motor-stop
  first, fail-closed guard against overlapping stacks).
- Platform quirks live in Claude's project memory and config comments:
  SBUS overrides USB-serial when the Taranis is on; CH340 udev collision
  (pin by KERNELS); firmware +X inverted (fixed in bridge, not TF);
  mecanum can't converge pure-rotation yaw tightly (PoseProgressChecker,
  yaw_goal_tolerance ≥ 0.40).
