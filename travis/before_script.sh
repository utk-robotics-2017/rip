#!/usr/bin/env bash
mkdir build
cd build

if [[ $2 ]]; then
	BRANCH = $1
else
	BRANCH = $3
fi

if [[ $BRANCH == "arduino_gen/"* ]]; then
	cmake -DARDUINO_GEN:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "pathfinder/"* ]]; then
	cmake -DPATHFINDER:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "roboclaw/"* ]]; then
	cmake -DROBOCLAW:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "cmd_messenger/"* ]]; then
	cmake -DCMD_MESSENGER:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "pathman/"* ]]; then
	cmake -DPATHMAN:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "navx/"* ]]; then
	cmake -DNAVX:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "communication/"* ]]; then
	cmake -DCOMMUNICATION:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else if [[ $BRANCH == "appendages/"* ]]; then
	cmake -DAPPENDAGES:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else
	cmake -DALL:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
fi