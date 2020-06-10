#!/bin/bash

echo "Generate and upload code coverage report"

ROS_WS=$HOME/target_ws
REPOSITORY_NAME=moveit

# Capture coverage info
lcov --capture --directory $ROS_WS --output-file coverage.info | grep -ve '^Processing'

# Extract repository files
lcov --extract coverage.info \"$ROS_WS/src/$REPOSITORY_NAME/*\" --output-file coverage.info | grep -ve '^Extracting'

# Filter out test files
lcov --remove coverage.info '*/test/*' --output-file coverage.info | grep -ve '^Removing'

# Output coverage data for debugging
lcov --list coverage.info

# Upload to codecov.io: -f specifies file(s) to upload and disables manual coverage gathering
bash <(curl -s https://codecov.io/bash) -f coverage.info -R $ROS_WS/src/$REPOSITORY_NAME
