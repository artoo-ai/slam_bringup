# vendor/

Third-party code we depend on but is not on PyPI / apt / rosdep. Copied
in verbatim so the repo is self-contained — `git clone` + `./install.sh`
fully builds the rover stack without a Google Drive scavenger hunt.

## Rosmaster_Lib_3.3.9/

Yahboom's Python driver for the YB-ERF01-V3.0 / ROSMASTER X3 STM32 board.
Used by `slam_bringup/yahboom_bridge_node.py` (see the *"Mecanum UGV -
GitHub - AutomaticAddison ROSMASTER X3 ROS2"* Obsidian note for Path A
rationale).

**Source:** Yahboom official Google Drive distribution for ROSMASTER X3
ROS2 — `For jetson orin super/py_install_V3.3.9.zip`. Drive link surfaces
from <https://www.yahboom.net/study/ROSMASTER-X3>.

**Version:** `3.3.9`. Bumping the version: drop a fresh
`py_install_VX.X.X.zip` next to this README, extract, replace the
`Rosmaster_Lib_X.X.X/` directory, and update `install.sh` to point at the
new path.

**License:** Yahboom does not include a license file in the distribution.
Treat as proprietary; do not redistribute outside this repo.

**Install:**

```bash
cd vendor/Rosmaster_Lib_3.3.9
pip3 install --user .
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK')"
```

`./install.sh` does this automatically as part of first-time setup on a
mecanum-platform Jetson.
