# `start_bench_tf.sh` — bench-fixture stopgap static TF

Publishes a single `livox_frame → camera_link` static TF so a Foxglove/RViz 3D panel can render Mid-360 + D435 clouds in the same view, BEFORE the URDF + `robot_state_publisher` chain is in place.

**Phase 1.7 makes this obsolete** — once the URDF publishes the full TF tree, you no longer need this script. Use `start_perception.sh` instead.

## What it does

A single `static_transform_publisher` from `livox_frame` (Mid-360) to `camera_link` (D435). X/Y/Z/RPY are constants at the top of the script — edit in place to match your rig's measured offset.

Idempotent — kills any prior bench-tf publisher before relaunching.

## When to use

- Quick sensor-co-visualization without bringing up the URDF.
- Recording a sensors-only bag and you want a TF to reproject in playback.

## When NOT to use

- Anything past Phase 1.7 — `start_perception.sh` and `start_slam.sh` publish the URDF tree which makes this redundant (and conflicting).

## Parameters

None — edit the X/Y/Z/RPY constants at the top of the script for your rig. Re-run after each edit (idempotency check swaps the new transform in cleanly).

## Examples

```bash
./start_bench_tf.sh    # publish whatever offsets are hard-coded
```

## See also

- [start_perception.md](start_perception.md) — URDF-driven TF tree (the proper fix)
- [start_sensors.md](start_sensors.md) — sensors only, no TF
