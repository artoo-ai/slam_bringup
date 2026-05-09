# Why SLAM Is Hard On Our Robot (And A Simpler Path Forward)

A research note on why a $500 robot vacuum maps a house in one pass while our
Livox MID-360 + D435i + Fast-LIO2 + RTAB-Map setup struggles in a single living
room, and a concrete plan to simplify.

---

## 1. The Core Question

> "My Roborock has a single-line lidar and maps the entire house in one pass.
> I have a 3D lidar, a depth camera, and two state-of-the-art SLAM packages,
> and I can't make a usable map of one room. How?"

The intuition that "more sensors + better algorithms = better maps" is wrong
in the same way that "more horsepower = faster lap times" is wrong. The
limiting factor is rarely raw capability — it is **how well the entire system
is matched to the problem you are trying to solve**.

The vacuum is solving a much easier problem with a much more tightly tuned
stack. We are solving a much harder problem with a much more general stack.
The capability gap is real but it cuts the opposite direction from intuition.

---

## 2. Why The Vacuum Works So Well

### 2.1 It solves a strictly 2D problem

A vacuum assumes:

- The floor is flat.
- The robot never pitches or rolls.
- The sensor stays at exactly one height.
- Motion is slow and planar (forward + yaw, no strafing, no Z motion).
- The environment is indoor, with walls, furniture legs, and doorways.

This lets it throw away the entire vertical axis. There is no roll/pitch/Z
drift to fight, no ground-plane segmentation, no 3D loop closure. The state
estimator tracks `(x, y, yaw)` — three numbers. Fast-LIO2 tracks 15+ states
including IMU biases, gravity, and full SO(3) orientation. **Every extra
state is a new way to drift.**

### 2.2 Its odometry is excellent

Roborocks have hard rubber wheels on hard floors with well-calibrated
encoders. Their dead reckoning is good enough that SLAM is mostly _correcting
small errors_, not _reconstructing motion from scratch_.

Our robot has **mecanum wheels**. Mecanum wheels:

- Slip diagonally on every strafe.
- Have rolling friction that varies with load and surface.
- Use a kinematic model that assumes no slip — which is always wrong.
- Produce odometry that drifts meters per minute under aggressive motion.

This is documented in our project memory (`project_mecanum_chassis_yaw.md`)
and is almost certainly the single largest contributor to our SLAM problems.
**SLAM is not magic — it cannot fix odometry that lies to it.** It can fight
bad odometry up to a point, but every bad odom update is a wrong prior the
optimizer has to overcome with sensor data.

### 2.3 It runs one purpose-built SLAM, not two general ones

The vacuum runs a single proprietary 2D graph-SLAM optimized for:

- Exactly that robot's dynamics.
- Exactly that lidar's height, range, and noise model.
- Exactly that motion profile (slow, indoor, planar).
- Aggressive loop closure tuned for small rooms.
- Tight wheel + IMU + lidar EKF, all calibrated at the factory.

We are running:

- **Fast-LIO2** — a lidar-inertial odometry system tuned by default for
  outdoor, fast-motion, large-scale environments (it was a research package
  for autonomous cars and drones before becoming popular for indoor).
- **RTAB-Map** — a visual + lidar SLAM with appearance-based loop closure,
  designed to be a generalist that works on many platforms.

These are two excellent packages. They are also **two systems that both want
to own the odometry frame**, both publish TFs, and both have hundreds of
parameters tuned for "reasonable defaults" rather than "our exact rover in
our exact living room." When you run them together you get a complex multi-
process pipeline where any one component being slightly wrong produces a
broken map.

### 2.4 Sensor placement is matched to the task

A vacuum lidar sits ~10 cm off the floor. At that height it sees:

- Chair legs.
- Table bases.
- Walls.
- Doorways.
- Sofa skirts.

Every one of those is a **dense, persistent, edge-rich feature** at the
height where navigation matters. The lidar gets a tight 2D slice through
exactly the geometry the robot has to avoid.

The Livox MID-360 mounted on top of our rover sees:

- Some walls (but they are smooth and featureless to a sparse 3D scanner).
- Some ceiling.
- Some floor.
- Mostly empty 3D space.

Sparse 3D point clouds in a small room with smooth walls give Fast-LIO2 very
little geometric structure to register scan-to-scan. The MID-360 was designed
for outdoor robotics where there is rich 3D structure — trees, buildings,
terrain. In a 4 m × 5 m living room, much of its capability is wasted.

### 2.5 They had years to tune it; we have one weekend at a time

Roborock has tuned their SLAM across millions of homes. We are running
defaults, with one rover, in one room.

---

## 3. What Is Specifically Hurting Us

A concrete list, roughly in order of likely impact.

### 3.1 Mecanum wheel odometry is bad input data

Already covered above. Our `yahboom_bridge` publishes wheel-odometry that is
optimistic about mecanum slip. Fast-LIO2 has IMU+lidar to fight back, but
every diagonal motion still poisons the prior.

**Research direction:** how much can we trust the mecanum odom? Should we
disable it entirely as a SLAM input and rely purely on lidar+IMU? Many
mecanum-platform SLAM setups do exactly this.

### 3.2 Two SLAM systems competing

Fast-LIO2 publishes its own odometry. RTAB-Map can consume external odometry
or compute its own. If both are publishing into the TF tree, or if RTAB-Map
is consuming Fast-LIO2 odom but applying its own corrections on top, you can
get TF fights, frame jitter, and inconsistent map updates.

Most working setups pick **one** of:

- Fast-LIO2 alone (3D map, no loop closure unless you add it externally).
- RTAB-Map alone with lidar (uses ICP odometry + RTAB-Map's loop closure).
- Fast-LIO2 as odometry source feeding RTAB-Map purely as a loop-closure
  layer (advanced; needs careful frame management).

**Research direction:** which integration mode are we currently running? Is
RTAB-Map subscribing to Fast-LIO2's odom, or computing its own?

### 3.3 IMU calibration and extrinsics

Fast-LIO2 needs:

- IMU intrinsic biases (gyro + accel) — usually estimated online but seeded
  from config.
- IMU-to-lidar extrinsic transform (rotation + translation) — must be exact.
  A 5° rotation error here will cause continuous drift.
- IMU noise parameters in the config matched to your specific IMU.

We have two IMUs in play (D435i internal + WitMotion). Which one feeds
Fast-LIO2? Are its extrinsics in the Fast-LIO2 config matched to the URDF?

**Research direction:** verify Fast-LIO2 IMU extrinsics against the URDF,
and verify which IMU is actually being used.

### 3.4 RTAB-Map default parameters are conservative

RTAB-Map's defaults assume general use. For a small indoor room, you usually
want:

- Smaller voxel sizes for occupancy grid.
- More aggressive loop closure thresholds.
- Tuned 2D projection bounds (z_min, z_max) that match the lidar height
  and the robot's footprint.
- Disabling memory management features that can prematurely forget
  recent scans in small environments.

### 3.5 3D-to-2D projection is lossy

To navigate, Nav2 wants a 2D occupancy grid. RTAB-Map can produce one by
projecting the 3D map down. This projection is sensitive to:

- The Z-range you project (too narrow misses obstacles, too wide includes
  ceiling/floor).
- Sensor noise (3D lidar floor returns become "obstacles" in 2D).
- Robot pitch/roll (a tilted scan projects to a curved wall).

A 2D lidar bypasses this problem entirely — its scan _is_ the 2D map.

---

## 4. The Proposed Simpler Approach

The principle: **match the stack to the problem**. We are trying to navigate
in 2D on a flat floor in a normal indoor environment. That is a 2D SLAM
problem. Solve it with 2D SLAM.

### 4.1 The plan

1. **Stop running Fast-LIO2 for now.** Not because it is bad, but because
   it adds a 3D state estimator we do not need to navigate a flat floor.
2. **Stop running RTAB-Map for now.** Same reason.
3. **Flatten the Livox MID-360 to a 2D scan** using the standard ROS package
   `pointcloud_to_laserscan`. This takes the 3D point cloud, slices a
   horizontal band at robot height, and emits a `sensor_msgs/LaserScan`
   indistinguishable to downstream code from a 2D lidar.
4. **Run `slam_toolbox` in `online_async` mode.** This is the de-facto
   standard 2D SLAM in ROS2. It is what almost every indoor wheeled robot
   running ROS2 uses. It handles loop closure, map saving, and pose
   correction for AMCL-style localization out of the box.
5. **Feed it the flattened scan + the existing `/odom` from yahboom_bridge.**
6. **Drive the rover slowly** and see if a map emerges.

### 4.2 What this tells us

If `slam_toolbox` produces a clean map: the problem was stack complexity and
parameter tuning, not fundamental sensor or odometry issues. We then have a
working 2D base we can navigate on while we figure out 3D mapping at our own
pace.

If `slam_toolbox` _also_ struggles: the problem is more fundamental —
probably odometry quality, IMU calibration, or TF tree issues. In that case,
no amount of 3D SLAM tuning will help. We would need to fix the foundation
(odometry, TF, calibration) before any SLAM stack can work.

This is a **diagnostic step as much as a solution.** It cleanly separates
"is our hardware/odometry fundamentally usable for SLAM" from "are we
running too complicated a stack."

### 4.3 Why this is the right next step

- **Smallest possible stack.** One SLAM node, one input topic, one output
  topic. Minimal places to be wrong.
- **Closest analog to what the vacuum does.** A 2D scan + 2D graph SLAM is
  exactly the vacuum's architecture.
- **Reversible.** All the Fast-LIO2 / RTAB-Map config stays in the repo. We
  are not deleting work, just choosing not to launch it for now.
- **Builds a foundation.** Once we have a working 2D map and Nav2 working
  on it, we can _add_ 3D mapping back as an enrichment layer rather than
  fighting a 4-component stack from day one.
- **Matches the documented best practice.** The standard ROS2 indoor mobile
  robot tutorial uses exactly this stack: Nav2 + slam_toolbox + 2D scan.

### 4.4 What we keep for later

- **Fast-LIO2 + Livox 3D** is the right tool for outdoor, multi-floor,
  large-scale mapping. Save it for when we take the rover outside or want a
  3D model of the workshop.
- **RTAB-Map + D435i** is the right tool for visual relocalization and
  textured 3D reconstruction. Save it for when we want photorealistic maps
  or visual loop closure.
- **D435i** still has value short-term as a forward-facing obstacle sensor
  for Nav2's local costmap, even while slam_toolbox owns the global map.

---

## 5. Concrete Next Steps For Research And Implementation

### 5.1 Things to read

- `slam_toolbox` README and parameter docs:
  https://github.com/SteveMacenski/slam_toolbox
- `pointcloud_to_laserscan` README:
  https://github.com/ros-perception/pointcloud_to_laserscan
- Nav2 docs on using slam_toolbox for online mapping.
- Steve Macenski's blog posts on slam_toolbox tuning for small rooms.

### 5.2 Things to verify in our setup

- Is `/odom` from `yahboom_bridge` actually publishing correctly while the
  rover moves? Plot it in Foxglove against a known short straight line and
  measure drift.
- Is the TF tree clean when no SLAM is running? `base_link` → `odom` should
  exist from the bridge. `map` should _not_ exist yet (slam_toolbox will
  create it).
- What is the actual height of the Livox above the floor? We need this for
  `pointcloud_to_laserscan` config (`min_height` / `max_height` band).

### 5.3 Implementation outline

1. Create a new launch file `launch/slam_2d.launch.py` that starts:
   - `pointcloud_to_laserscan` node converting `/livox/lidar` →
     `/scan`.
   - `slam_toolbox` in `online_async` mode subscribed to `/scan` and
     `/odom`.
2. Create a corresponding `start_slam_2d.sh` and `kill_slam_2d.sh`.
3. Drive the rover in a slow square in the living room.
4. Save the resulting map with `slam_toolbox`'s `save_map` service.
5. Compare the result qualitatively against what we get from Fast-LIO2 +
   RTAB-Map after the same drive.

### 5.4 Success criteria

A "good enough" map for the simplification experiment:

- Walls of the living room visible as connected lines in the occupancy grid.
- No drift such that the same wall appears as two parallel lines.
- Doorways and major furniture identifiable.
- Map can be loaded by Nav2 + AMCL and the rover can be localized in it
  while standing still.

If we hit those criteria, we have a working baseline. Everything else is
improvement on top.

---

## 6. Open Questions Worth Researching

- How bad is our mecanum odometry, _quantitatively_? Without a number, we
  are guessing.
- Should we add a wheel-slip compensation layer (e.g. fuse wheel odom with
  IMU yaw via `robot_localization`'s EKF) before feeding it into SLAM?
- Is the WitMotion IMU good enough to be the primary IMU, or should we use
  the D435i's BMI085? Which one is Fast-LIO2 currently using?
- Could we mount the Livox lower so its horizontal slice catches more
  furniture geometry, the way a vacuum lidar does?
- For the 2D-flattening band, what Z range works best in our living room —
  do we slice at 20–40 cm (chair leg height) or higher?

---

## 7. Mecanum Wheels: Replace Or Software-Fix?

### 7.1 Current state in our stack (important nuance)

The current `yahboom_bridge` **does not publish wheel odometry at all** — it
is a `cmd_vel`-only bridge. Per the bridge's own header comment, telemetry
(encoder odom, IMU) was deliberately omitted because FAST-LIO2 owns
`/Odometry`.

Implication: in the **current FAST-LIO2 + RTAB-Map stack**, mecanum wheel
odometry is *not* directly poisoning SLAM. The wheels still hurt us
indirectly — slip-induced jolts and yaw drift confuse the IMU and lidar
registration — but no raw wheel pose is being fed to the optimizer.

The wheel-odom question becomes load-bearing as soon as we move to the
**simpler 2D `slam_toolbox` stack**, because `slam_toolbox` benefits
enormously from a clean `/odom` prior. There, the choice is real: do we
trust mecanum odom enough to feed it in?

### 7.2 The three options, in order of cost

#### Option A — Keep mecanum, drop wheel odom from SLAM (free, reversible)

- Run `slam_toolbox` in pure **scan-matching** mode (no `/odom` input).
- Lidar + IMU carries pose; no wheel odom is trusted.
- Cost: nothing. Zero hardware change, zero new code. Just a launch flag.
- Risk: less robust in featureless rooms (long blank walls). Scan matching
  can lose track in geometrically empty environments.
- This is the standard first move for mecanum SLAM platforms.

#### Option B — Keep mecanum, fuse wheel odom with heavy IMU weighting (recommended default)

- Add `robot_localization`'s EKF.
- Fuse: WitMotion IMU (high weight on yaw) + yahboom encoder odom (high
  covariance on linear, very high on lateral) → produces a synthetic
  `/odom`.
- IMU effectively owns yaw; wheels contribute only a weak forward-velocity
  prior.
- Cost: an afternoon of config. Reversible.
- Risk: mostly tuning the covariance matrix.
- This is what most production mecanum platforms do.

#### Option C — Replace wheels with rubber + diff-drive (hardware change)

- Buy: matching diameter rubber wheels, possibly different motors/encoders
  if Yahboom hubs aren't compatible, mounting hardware.
- Lose: holonomic motion (no strafe).
- Gain: clean dead reckoning, every ROS tutorial works out of the box, no
  more frame-inversion hacks.
- Cost: real money + a chassis hack day. Hard to undo.
- Worth it only if the rover's primary job is indoor mapping/navigation
  and we don't actually use strafing.

### 7.3 Recommendation

**Don't buy wheels yet.** Try the cheap, reversible answer first:

1. Build the 2D `slam_toolbox` stack with a `use_wheel_odom` parameter
   (see §8 below).
2. Run with `use_wheel_odom:=false` first → pure scan-matching. If maps
   come out clean, mecanum slip was never the SLAM bottleneck.
3. If scan-matching is too fragile, flip to `use_wheel_odom:=true` with
   the EKF pre-fusion (Option B). Tune IMU vs wheel covariance.
4. Only consider hardware swap (Option C) if both options fail to produce
   a stable map *and* we conclude the platform's primary job is indoor
   nav, not holonomic demos.

> Mecanum is not the wrong wheel for SLAM — *trusting mecanum wheel
> odometry* is the wrong default. Plenty of mecanum platforms map fine;
> they just don't feed wheel odom raw into the optimizer.

---

## 8. Plan: `use_wheel_odom` Parameter In The 2D Stack

The proposed `launch/slam_2d.launch.py` (from §4.3) will expose:

- `use_wheel_odom` (default `false`) — when false, `slam_toolbox` runs in
  pure scan-matching mode with no `/odom` subscription. When true, it
  subscribes to `/odom` from the EKF (or directly from yahboom_bridge once
  it publishes one).
- `use_imu_ekf` (default `true`) — when true, launch `robot_localization`
  EKF fusing WitMotion IMU + (optionally) wheel odom into a synthetic
  `/odom`. When false, skip the EKF; `/odom` comes directly from whatever
  publishes it.

Prerequisite work before this parameter does anything useful:

1. Extend `yahboom_bridge_node.py` to optionally publish `/yahboom/odom`
   from encoder telemetry. Keep it on a namespaced topic (per the existing
   comment: *"If you ever want the Yahboom IMU as a fallback, expose it on
   a namespaced topic (/yahboom/imu) so it doesn't conflict"*). New launch
   arg `publish_odom:=false` default — opt in only.
2. Configure `robot_localization` EKF with sensible covariances for
   mecanum (high lateral covariance, high linear-acceleration covariance,
   IMU yaw trusted).
3. Wire all three into `slam_2d.launch.py` so the user can flip
   combinations from a single command.

Decision matrix for testing:

| `use_wheel_odom` | `use_imu_ekf` | What gets fed to slam_toolbox |
|---|---|---|
| false | false | nothing (pure scan-matching) ← simplest, try first |
| false | true | IMU-only EKF /odom (yaw + integrated accel) |
| true | true | full EKF /odom (IMU + wheel encoders) ← Option B |
| true | false | raw yahboom encoder /odom ← worst case for mecanum |

Clean experiment grid to find the minimum that produces a usable map,
without committing to hardware changes prematurely.

---

## 9. IMU Choice For The 2D Stack (And Why The "Pair With Source" Rule Doesn't Apply Here)

The advice "always use the IMU on the same board as the lidar" comes from
**Fast-LIO2 specifically**, and is about **lidar scan deskewing**. A Livox
sweep takes ~100 ms; during that window the robot moves, and Fast-LIO2
needs IMU samples *inside the scan window* — sharing a clock domain with
the lidar firmware — to estimate the per-point motion correction. Even
5–10 ms of IMU↔lidar clock skew warps the deskewed cloud and drifts the
map.

`slam_toolbox` + `robot_localization` is a **fundamentally different
pipeline**:

- Input is a flattened `LaserScan` at ~10 Hz, already coherent within a
  single scan at indoor speeds.
- The EKF fuses at 30 Hz using ROS2 timestamps with TF interpolation.
- Tolerable timestamp jitter: tens of milliseconds. Roughly 1000× looser
  than Fast-LIO2's deskewing requirement.

In this 2D stack, any IMU works. Choose by quality and convenience, not
timing pairing.

### Comparison

| IMU | Pros | Cons |
|---|---|---|
| Mid-360 internal (`/livox/imu`) | Published while lidar runs. Rigidly mounted to lidar — extrinsic ≈ 0. ICM-40609 6-axis. One less USB device. | 6-axis (no magnetometer). Tied to lidar driver lifecycle. |
| D435i internal (`/d435_front/.../imu`) | Solid BMI085. Hardware-synced to camera. | Only publishes while camera streams. Lever-arm extrinsic must be verified. Stops if D435 power-cycles. |
| WitMotion 901 (`/imu/data`) | 9-axis with magnetometer. Independent of sensor lifecycle. Already configured. | Extra USB CH340 device — YB-ERF01 collision risk per project memory. Driver-stamped, not hardware-stamped. |

### Recommendation: Mid-360 IMU for the 2D EKF

1. Publishes whenever the lidar is up — no extra driver to babysit.
2. Rigidly mounted to the device producing the scan — extrinsic is the
   Livox factory cal already in `fast_lio_mid360.yaml`.
3. One less USB device fighting the WitMotion + YB-ERF01 CH340 pairing.
4. We don't need 9-axis — `slam_toolbox` corrects yaw drift via scan
   matching at every keyframe.

WitMotion stays in the rig as a backup IMU but isn't the default for SLAM.

### Timing hygiene (no special "fix" needed)

- All three drivers stamp `header.stamp` on the host. NTP'd system clock
  keeps them coherent.
- `robot_localization` `transform_timeout: 0.1` handles small lag.
- IMU rate (100–200 Hz) ≫ EKF rate (30 Hz), so any single late/dropped
  sample is invisible.
- Set `imu0_relative: true` so EKF zeros initial yaw at startup.

> **Note on Option A:** `rf2o_laser_odometry` is laser-only — it derives
> motion from successive scans. Option A doesn't use any IMU at all. The
> IMU choice only matters for Option B's EKF.

---

## 10. The Big Picture Lesson

> The vacuum's advantage is not better hardware or better algorithms. It is
> a tighter match between the problem, the sensor, the motion model, and the
> SLAM tuning. We can have all of that too — but we have to build it
> deliberately, one layer at a time, instead of stacking up the most
> capable components and hoping they cooperate.

Start simple. Get a map. Then add capability back, one layer at a time, with
each layer measurably improving on the last.
