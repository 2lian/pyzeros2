#!/usr/bin/env bash

set -u

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$workspace_dir"

failed=0

run_tests() {
    local environment="$1"
    shift

    echo
    echo "=== ${environment}: pixi run -e ${environment} $* ==="
    if pixi run -e "$environment" "$@"; then
        echo "=== ${environment}: PASS ==="
    else
        echo "=== ${environment}: FAIL ==="
        failed=1
    fi
}

# Python-only environments: the test task excludes ROS interop tests.
run_tests py311 pytest
run_tests py312 pytest
run_tests py313 pytest
run_tests py314 pytest

# ROS environments: run the complete suite, including interop tests.
run_tests jazzy pytest
run_tests lyrical pytest

echo
if [[ "$failed" -eq 0 ]]; then
    echo "All test environments passed."
else
    echo "One or more test environments failed."
fi

exit "$failed"
