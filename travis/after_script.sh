#!/usr/bin/env bash
cd build
lcov --directory . --capture --output-file coverage.info # capture coverage info
lcov --remove coverage.info '/usr/*' --output-file coverage.info # filter out system


if [[ $2 ]]; then
	BRANCH = $1
else
	BRANCH = $3
fi

if [[ $BRANCH == "arduino_gen/"* ]]; then
	lcov --remove coverage.info '/utilities/*' --output-file coverage.info # filter out utils
else if [[ $BRANCH == "pathfinder/"* ]]; then
	lcov --remove coverage.info '/utilities/*' --output-file coverage.info # filter out utils
else if [[ $BRANCH == "roboclaw/"* ]]; then
else if [[ $BRANCH == "cmd_messenger/"* ]]; then
else if [[ $BRANCH == "pathman/"* ]]; then
else if [[ $BRANCH == "navx/"* ]]; then
	lcov --remove coverage.info '/utilities/*' --output-file coverage.info # filter out utils
else if [[ $BRANCH == "communication/"* ]]; then
	lcov --remove coverage.info '/utilities/*' --output-file coverage.info # filter out utils
else if [[ $BRANCH == "appendages/"* ]]; then
	lcov --remove coverage.info '/utilities/*' --output-file coverage.info # filter out utils
fi

lcov --list coverage.info #debug info
