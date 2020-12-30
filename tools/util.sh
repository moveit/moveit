#!/bin/bash

# Software License Agreement - BSD 3-Clause License
#
# Author:  Robert Haschke
#
# Adapted from script in moveit_ci
# https://github.com/ros-planning/moveit_ci/blob/master/util.sh

export ANSI_RED="\033[31m"
export ANSI_GREEN="\033[32m"
export ANSI_YELLOW="\033[33m"
export ANSI_BLUE="\033[34m"

export ANSI_THIN="\033[22m"
export ANSI_BOLD="\033[1m"

export ANSI_RESET="\033[0m"
export ANSI_CLEAR="\033[0K"

# usage: echo -e $(colorize RED Some ${fancy} text.)
function colorize() {
   local color reset
   while true ; do
      case "${1:-}" in
         RED|GREEN|YELLOW|BLUE)
            color="ANSI_$1"; eval "color=\$$color"; reset="${ANSI_RESET}" ;;
         THIN)
            color="${color:-}${ANSI_THIN}" ;;
         BOLD)
            color="${color:-}${ANSI_BOLD}"; reset="${reset:-${ANSI_THIN}}" ;;
         *) break ;;
      esac
      shift
   done
   echo -e "${color:-}$@${reset:-}"
}

_have_fixes() {
    if ! git diff --quiet . ; then  # check for changes in current dir
        echo -e $(colorize RED "\\nThe following issues were detected:")
        git --no-pager diff .
        return 0
  fi
  return 1
}

function _collect_modified_files() {
  local -n __modified_files=$1     # -n to modify argument by reference
  local filter=$2
  local src_dir=${3:-$PWD}
  local base=${4:-$BASE_BRANCH}

  # Find top-level git folder of src_dir
  local prefix_dir=$(cd "$src_dir"; git rev-parse --show-toplevel)
  # Strip git folder from src_dir to keep relative path from git root to source files as stip_prefix
  strip_prefix="${src_dir#$prefix_dir/}"

  pushd $prefix_dir > /dev/null
  while IFS='' read -r line ; do
    # Add modified or added files to array - only using the relative path from src_dir, i.e. removing strip_prefix
    __modified_files+=("${prefix_dir}/${line}")
  done < <(git diff --name-only --diff-filter=MA "$base"..HEAD "$src_dir" | grep "$filter")
  popd > /dev/null
}
