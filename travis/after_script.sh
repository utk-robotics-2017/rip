#!/usr/bin/env bash
set -e
cd build

lcov --directory . --capture --output-file coverage.info # capture coverage info
lcov --remove coverage.info '/usr/*' --output-file coverage.info # filter out system

if [[ $2 ]]; then
    BRANCH=$1
else
    BRANCH=$3
fi

# REVIEW: Do we need any of this?

if [[ $BRANCH == "arduino_gen/"* ]]; then
    lcov --remove coverage.info '/external/*' --output-file coverage.info # filter out utils
elif [[ $BRANCH == "pathfinder/"* ]]; then
    lcov --remove coverage.info '/external/*' --output-file coverage.info # filter out utils
elif [[ $BRANCH == "roboclaw/"* ]]; then
    echo "TODO: lcov for roboclaw"
elif [[ $BRANCH == "cmd_messenger/"* ]]; then
    echo "TODO: lcov for cmd_messenger"
elif [[ $BRANCH == "pathman/"* ]]; then
    echo "TODO: lcov for pathman"
elif [[ $BRANCH == "navx/"* ]]; then
    lcov --remove coverage.info '/external/*' --output-file coverage.info # filter out utils
elif [[ $BRANCH == "communication/"* ]]; then
    lcov --remove coverage.info '/external/*' --output-file coverage.info # filter out utils
elif [[ $BRANCH == "appendages/"* ]]; then
    lcov --remove coverage.info '/external/*' --output-file coverage.info # filter out utils
fi

lcov --list coverage.info #debug info

bash <(curl -s https://codecov.io/bash) -t 233dc217-113d-4ab4-9db7-cfd9f85e41c7 || echo "Codecov did not collect coverage reports"
