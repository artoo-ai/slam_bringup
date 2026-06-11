# slam_bringup — project instructions

## Daily engineering log (required practice)

Maintain `docs/DAILY_LOG.md` as work happens, not after the fact:

- When a problem appears, add it to today's entry as *Symptom →
  Investigation → Root cause → Fix → Status* (newest day first).
- Record dead ends and refuted hypotheses too — they're half the story.
- Reference commits by hash. End each day with "Open threads" checkboxes
  so the next session knows where to pick up.
- Symptom-indexed quick fixes also go in `docs/troubleshooting.md`
  (add an Index line); the log holds the narrative, troubleshooting
  holds the recipe.

## Things that bite (verify before re-tuning)

- Rotation is capped at 0.6 rad/s everywhere in `nav2_params_2d.yaml` —
  it's a SLAM constraint (rf2o can't track faster spins of the Mid-360's
  aggregated scan), not a motion-tuning choice. Don't raise it.
- Config/launch edits do nothing on the robot until `./build.sh` — when
  debugging, check DEPLOYED values (`ros2 param get …`), not repo files.
- This Mac repo and the Jetson's checkout drift; confirm the robot is
  running the commit you think it is before interpreting test results.
- `urdf/mecanum.urdf.xacro` `base_link_to_plate_top` and
  `PLATFORM_BRIDGES['mecanum']` in `launch/slam.launch.py` must move in
  lockstep (see comments in both files).
- Never run RViz on the Jetson during real sessions; `sudo
  jetson_clocks` first.
