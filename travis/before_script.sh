#!/usr/bin/env bash
mkdir build
cd build

if [[ $2 ]]; then
	BRANCH=$1
else
	BRANCH=$3
fi

if [[ $BRANCH == "arduino_gen/"* ]]; then
	cmake -G"Unix Makefiles" -DARDUINO_GEN:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "pathfinder/"* ]]; then
	cmake -DPATHFINDER:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "roboclaw/"* ]]; then
	cmake -DROBOCLAW:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "cmd_messenger/"* ]]; then
	cmake -DCMD_MESSENGER:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "pathman/"* ]]; then
	cmake -DPATHMAN:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "navx/"* ]]; then
	cmake -DNAVX:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "communication/"* ]]; then
	cmake -DCOMMUNICATION:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
elif [[ $BRANCH == "appendages/"* ]]; then
	cmake -DAPPENDAGES:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
else
	cmake -DALL:BOOL=ON -DENABLE_TESTING:BOOL=ON ..
fi
