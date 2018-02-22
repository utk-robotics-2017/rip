#!/usr/bin/env bash
set -e

pushd build

# Checks if it compiles
make -j$(nproc )

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

function test_motor_controllers() {
    pushd core/motor_controllers
    ./motor_controllers_test
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
    test_motor_controllers
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
    test_peripherycpp
    test_motor_controllers
    ;;
esac

### Done with tests
popd
