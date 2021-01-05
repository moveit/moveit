# MoveIt tools

This documents tools for local lint testing and CI.

## Build and install for local use

Create workspace to build the ament packages.

    mkdir -p /opt/ws_ament_lint/src
    cd /opt/ws_ament_lint
    vcs import src --input https://raw.githubusercontent.com/tylerjw/moveit/clang_tidy_ament_script/tools/ament_lint.repos

Build the workspace

    colcon build

Add this line to your `.bashrc`:

    source /opt/ws_ament_lint/install/setup.bash

## Run ament_clang_format

Run this command from the root of the repo to check if your code is formatted correctly.  Make sure you have installed clang-format-10.

    ament_clang_format --config ./.clang-format --clang-format-version 10

To have it reformat your code inplace use the `--reformat` flag.

## Run ament_clang_tidy

`clang-tidy` is a static analysis linter that takes a long time to execute.  As a result we limit testing to packages that have changed in any given PR.  To run a similar test locally you need to first build your project with this option:

    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

Then run `ament_clang_tidy` from the workspace directory like what follows.

    ament_clang_tidy --filter-repo-path src/moveit --filter-diff master

To have it attempt to fix the code in place you can use the option `--reformat`.

### Background

This is using a fork of ament_lint by @tylerjw for use of ament_clang_tidy and ament_clang_format.  The goals of this is is:

* Standaone script that can be run locally, and therefore CI provider/system agnostic
* Has options for filtering clang-tidy tests to packages in a repo
* Has options for filtering clang-tidy tests to packages with changes since some point in git
* Collaberate with others in the ROS community to contribute these features so they can be used more widely

Discussion of these changes can be found in these PRs to ament_lint:

* [Add --clang-format-version option](https://github.com/ament/ament_lint/pull/282)
* [Add filtering to ament_clang_tidy](https://github.com/ament/ament_lint/pull/280)

Quoting @tfoote as to why the ament_clang_tidy change is unlikely to be accepted:
> For your use case I would suggest that you bump up a layer and instead of running ament_clang_tidy with a filter so that it skips running for other packages instead you use the build tool to select the packages from that repository and selectively invoke ament_clang_tidy explicitly on the packages you want to test. (Having used colcon list or other mechanism's to explicitly find said list based on the repository).

I do not yet have a solution that works at the build tool level (catkin or colcon).  I do not want to make MoveIt have a source dependency on ament.
