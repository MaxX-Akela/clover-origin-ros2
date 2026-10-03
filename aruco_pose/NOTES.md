# Notes on the ROS 2 port

Observations about the original code that were intentionally **not** changed (the port keeps the algorithms as is).

- `aruco_map.cpp`, `callback()`: `markers->markers.empty()` is checked twice; the second check
  (`goto publish_debug`) is unreachable, so the debug image is never published when no markers are detected.
- `aruco_detect.cpp`: `enabled_ = enabled && length > 0`, so with `estimate_poses: false` and no `length`
  the detector is silently disabled.
- `aruco_map.cpp`, `createGridBoard()`: marker ids are indexed as `marker_ids[y * markers_y + x]`,
  which is only correct for square grids (`markers_x == markers_y`); `y * markers_x + x` is expected.
- `aruco_map.cpp`, `createGridBoard()`: an explicitly empty `marker_ids` array is treated as "not set".
- `utils.h`, `isFlipped()`: `abs()` is used on doubles; it relies on the `<cmath>` overload.
- `aruco_map` has no `SetMarkers` service in the original (only `aruco_detect/set_length_override` does).
