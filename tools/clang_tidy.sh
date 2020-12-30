#!/bin/bash

# Software License Agreement - BSD 3-Clause License
#
# Author:  Robert Haschke and Tyler Weaver
#
# Adapted from script in moveit_ci
# https://github.com/ros-planning/moveit_ci/blob/master/check_clang_tidy.sh
#
# Intended to be run from workspace directory.  Takes one argument, the base
# git tag or branch to compare against for finding file changes.  Leaving
# that argument unchanged defaults it to master.

WS_DIR=$PWD
REPO_DIR="${WS_DIR}/src/moveit"
TOOLS_DIR="${REPO_DIR}/tools"
BASE_BRANCH=${1:-master}

# source utility functions
source ${TOOLS_DIR}/util.sh

_run_clang_tidy_fix() {
    SOURCE_PKGS=$(catkin_topological_order $REPO_DIR --only-names 2> /dev/null)
    PKG_PATHS=()
    for pkg in ${SOURCE_PKGS[@]} ; do
        echo -e "- $(colorize BLUE Processing $pkg)"
        file="$WS_DIR/build/$pkg/compile_commands.json";
        build_dir=$(dirname "$file")
        if [ -r "$file" ]; then

            modified_files=()
            src_dir=$(grep "^CMAKE_HOME_DIRECTORY:INTERNAL=" "${build_dir}/CMakeCache.txt")
            _collect_modified_files modified_files "\.cpp$" $(realpath "${src_dir#*=}") $BASE_BRANCH

            if [ ${#modified_files[@]} -eq 0 ]; then
                echo "No modified .cpp files"
            else
                PKG_PATHS+=("${file}")
                cmd="clang-tidy-6.0 \
                    -fix -header-filter=\"${REPO_DIR}/.*\" -p ${build_dir} \
                    ${modified_files[@]:-}"
                echo $cmd
                $cmd 2> /dev/null
            fi
        else
            echo $(colorize YELLOW "$file not found, skipping")
        fi
    done

    pushd $REPO_DIR > /dev/null
    # if there are workspace changes, print broken pkg to file descriptor 3
    _have_fixes && 1>&3 ezcho $pkg || true
    popd > /dev/null
}

# Make sure no changes have occured in repo
pushd $REPO_DIR > /dev/null
if ! git diff-index --quiet HEAD --; then
    # changes
    read -p "You have uncommitted changes, are you sure you want to continue? (y/n)" resp

    if [[ "$resp" != "y" ]]; then
        exit -1
    fi
fi
popd > /dev/null

# run the test
echo -e $(colorize BLUE "Running clang-tidy test on files changed \
    since branch \`$BASE_BRANCH\` in \`$REPO_DIR\`")
3>/tmp/clang-tidy.tainted _run_clang_tidy_fix
result=$?
test $result -ne 0 && exit $result

# Read content of /tmp/clang-tidy.tainted into variable TAINTED_PKGS
TAINTED_PKGS=$(< /tmp/clang-tidy.tainted)

if [ -z "$TAINTED_PKGS" ] ; then
  echo -e $(colorize GREEN "Passed clang-tidy check")
else
  echo -e "$(colorize RED \"clang-tidy check failed for the following packages:\")\\n$(_colorize YELLOW $(colorize THIN $TAINTED_PKGS))"
  exit 2
fi
