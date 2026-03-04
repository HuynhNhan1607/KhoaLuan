#!/bin/bash
set -e

# Nhận tham số robot ID từ command line (robot1 hoặc robot2)
ROBOT_ID=${1:-robot1}  # Mặc định là robot1 nếu không truyền tham số

if [ "$ROBOT_ID" != "robot1" ] && [ "$ROBOT_ID" != "robot2" ]; then
    echo "Error: Invalid robot ID. Use 'robot1' or 'robot2'"
    echo "Usage: ./build.sh [robot1|robot2]"
    exit 1
fi

echo "Building project for $ROBOT_ID..."

# Convert robot1 -> 1, robot2 -> 2 for compile-time define
ROBOT_NUM=${ROBOT_ID//robot/}

gcc -Wall -Wextra -std=c11 \
    -D_GNU_SOURCE -D_POSIX_C_SOURCE=200809L \
    -DROBOT_ID=$ROBOT_NUM \
    -I./inc \
    -o single_mini-server ./src/*.c ./database/*.c \
    -lsqlite3 -lpthread -lm -lgpiod

echo "Build done for $ROBOT_ID! Run with ./single_mini-server"