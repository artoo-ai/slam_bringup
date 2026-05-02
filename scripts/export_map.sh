#!/usr/bin/env bash
# Export an RTABMap database to a CloudCompare-friendly PLY.
#
# Defaults are tuned for SLAM-debug visualization on this stack:
#   --scan              : assembled LiDAR cloud (dense, what you actually want)
#                         not the default occupancy-grid voxels which look
#                         like a sparse trail of dots in CloudCompare.
#   --cam_projection    : project D435 keyframe RGB onto the LiDAR points so
#                         the cloud opens with photo-realistic color.
#   --max_range 30      : drop noisy returns past 30 m. Indoor maps don't
#                         need farther range and far points are mostly noise.
#   --voxel 0.03        : 3 cm voxel grid. Cuts the PLY size by ~10× while
#                         keeping every wall feature an indoor map needs.
#                         Use --voxel-size for a custom value.
#
# Output goes to ~/maps/<timestamp>/cloud.ply by default. Override with
#   ./scripts/export_map.sh --db PATH --output DIR --voxel-size N
#
# Usage:
#   ./scripts/export_map.sh                                  # defaults
#   ./scripts/export_map.sh --voxel-size 0.01                # max detail
#   ./scripts/export_map.sh --db ~/.ros/rtabmap.db.bak       # alternate DB
#   ./scripts/export_map.sh --no-color                       # skip cam_projection (faster, no RGB)

set -euo pipefail

DB="$HOME/.ros/rtabmap.db"
OUTPUT="$HOME/maps/$(date +%Y%m%d-%H%M%S)"
VOXEL_SIZE="0.03"
MAX_RANGE="30"
WITH_COLOR=1

while [ $# -gt 0 ]; do
  case "$1" in
    --db)         DB="$2"; shift 2 ;;
    --output)     OUTPUT="$2"; shift 2 ;;
    --voxel-size) VOXEL_SIZE="$2"; shift 2 ;;
    --max-range)  MAX_RANGE="$2"; shift 2 ;;
    --no-color)   WITH_COLOR=0; shift ;;
    -h|--help)
      grep -E '^# ' "$0" | sed 's/^# //;s/^#//'
      exit 0
      ;;
    *)
      echo "export_map: unknown option '$1'. Try --help." >&2
      exit 1
      ;;
  esac
done

if [ ! -f "$DB" ]; then
  echo "export_map: ERROR — database not found: $DB" >&2
  echo "  Pass a different path with --db, or run ./start_slam.sh first" >&2
  echo "  to record a map (default location is ~/.ros/rtabmap.db)." >&2
  exit 1
fi

mkdir -p "$OUTPUT"

CMD=(rtabmap-export
     --scan
     --max_range "$MAX_RANGE"
     --voxel     "$VOXEL_SIZE"
     --output_dir "$OUTPUT"
)
[ "$WITH_COLOR" -eq 1 ] && CMD+=(--cam_projection)
CMD+=("$DB")

echo "==> ${CMD[*]}"
"${CMD[@]}"

echo ""
echo "Export complete. Files:"
ls -lh "$OUTPUT"
echo ""
echo "Open in CloudCompare:"
echo "  open '$OUTPUT'/cloud.ply           # macOS"
echo "  cloudcompare '$OUTPUT'/cloud.ply   # Linux"
