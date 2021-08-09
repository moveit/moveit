#!/bin/bash

# Custome BUILDER to build in devel-space only

BUILDER=catkin_tools ici_source_builder

function ici_extend_space {
  echo "$1/devel"
}

function _catkin_config {
    local extend=$1; shift
    local ws=$1; shift
    ici_exec_in_workspace "$extend" "$ws" catkin config --init
}
