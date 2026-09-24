#!/usr/bin/env bash
# ==============================================================================
# Run the project's test suite exactly the way CI does.
#
#   ./run_tests.sh            # everything available on this machine
#   ./run_tests.sh hermetic   # numpy-only suites (no PyQt5, no ROS 2)
#
# The hermetic set is what .github/workflows/tests.yml gates merges on; the
# full set additionally runs the PyQt5 import smoke tests.
# ==============================================================================
set -e
cd "$(dirname "$0")"

HERMETIC="test_health test_telemetry test_path_planner test_map_eval \
test_scaling test_audio"

if [ "${1:-all}" = "hermetic" ]; then
    exec python3 -m unittest ${HERMETIC} -v
fi

# test_gui_layout is last: it imports drone_gcs, whose module body re-execs
# the interpreter to inject ROS 2 library paths.
python3 -m unittest ${HERMETIC} test_smoke test_statustext test_video_sources \
    test_rviz_palette test_gui_features test_planner_worker test_gui_layout -v
