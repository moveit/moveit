#!/bin/bash
set -e

# setup ros environment
source "/root/ws_moveit/devel/setup.bash"
exec "$@"
