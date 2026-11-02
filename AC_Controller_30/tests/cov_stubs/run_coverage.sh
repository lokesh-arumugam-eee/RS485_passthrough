#!/bin/bash
# run_coverage.sh — one-command coverage build + HTML report
# Run from WSL inside the tests/cov_stubs/ directory:
#   cd /mnt/d/Projects/Modbus_project/Modbus_cpplib_S3/AC_Controller_30/tests/cov_stubs
#   bash run_coverage.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Installing tools if missing ==="
NEED_INSTALL=0
command -v make  &>/dev/null || NEED_INSTALL=1
command -v gcc   &>/dev/null || NEED_INSTALL=1
command -v g++   &>/dev/null || NEED_INSTALL=1
command -v lcov  &>/dev/null || NEED_INSTALL=1
command -v genhtml &>/dev/null || NEED_INSTALL=1

if [ "$NEED_INSTALL" -eq 1 ]; then
    sudo apt-get update -qq
    sudo apt-get install -y build-essential gcc g++ lcov
fi

echo "=== Cleaning previous build ==="
make clean 2>/dev/null || true

echo "=== Building and running coverage harness ==="
make all

echo ""
echo "=== Coverage complete! ==="
echo "Open file://$(readlink -f html_report/index.html) in a browser"
echo "Or from Windows: file:///$(wslpath -w html_report/index.html 2>/dev/null || echo 'html_report/index.html')"
