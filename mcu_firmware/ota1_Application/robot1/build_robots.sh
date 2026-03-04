#!/bin/bash
# Build script for multiple robots from single source code
# Usage: ./build_robots.sh [robot_id]
#   - No args: build both robot1 and robot2
#   - With arg: build only specified robot (1 or 2)

set -e

# Source ESP-IDF environment
source ~/esp/v5.5.2/esp-idf/export.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

build_robot() {
    local id=$1
    echo "=========================================="
    echo "  Building Robot ${id}"
    echo "=========================================="
    
    idf.py -B build_robot${id} -DROBOT_ID=${id} build
    
    # Copy binary to project root
    cp build_robot${id}/robot${id}.bin ./robot${id}.bin
    
    echo "✓ robot${id}.bin created"
}

# Parse arguments
if [ $# -eq 0 ]; then
    # Build both
    build_robot 1
    build_robot 2
    echo ""
    echo "=========================================="
    echo "  Build Complete!"
    echo "=========================================="
    echo "Output files:"
    echo "  - robot1.bin"
    echo "  - robot2.bin"
else
    build_robot $1
fi
