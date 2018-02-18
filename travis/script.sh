#!/usr/bin/env bash
set -e

pushd build

# Checks if it compiles
make -j$(nprocs )

if [[ $2 ]]; then
    BRANCH=$1
else
    BRANCH=$3
fi

function test_arduino_gen() {
    pushd arduino_gen
    ./arduino_gen_test
    popd
}

function test_roboclaw() {
    pushd core/roboclaw
    ./roboclaw_test
    popd
}

function test_cmd_messenger() {
    pushd core/utilities/cmd_messenger
    ./cmd_messenger_test
    popd
}

function test_peripherycpp() {
    pushd core/utilities/peripherycpp
    ./peripherycpp_test
    popd
}

case $BRANCH in
  arduino_gen*)
    test_arduino_gen
    ;;
  roboclaw*)
    test_roboclaw
    ;;
  cmd_messenger*)
    test_cmd_messenger
    ;;
  periphery*)
    test_peripherycpp
    ;;
  *)
    test_arduino_gen
    test_cmd_messenger
    test_roboclaw
    test_peripherycpp
    ;;
esac

### Done with tests
popd
