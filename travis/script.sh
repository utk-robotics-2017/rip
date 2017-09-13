#!/usr/bin/env bash

cd build

# Checks if it compiles
make -j4

if [[ $2 ]]; then
	BRANCH=$1
else
	BRANCH=$3
fi

# Runs cpp check
if [[ $BRANCH == "arduino_gen/"* ]]; then
	cd arduino_gen
	./arduino_gen_test
	cd ../../arduino_gen
elif [[ $BRANCH == "pathfinder/"* ]]; then
	cd pathfinder
	./pathfinder_test
	cd ../../core/navigation/pathfinder
elif [[ $BRANCH == "roboclaw/"* ]]; then
	cd roboclaw
	./roboclaw_test
	cd ../../utilities/roboclaw
elif [[ $BRANCH == "cmd_messenger/"* ]]; then
	cd cmd_messenger
	./cmd_messenger_test
	cd ../../utilities/cmd_messenger
elif [[ $BRANCH == "pathman/"* ]]; then
	cd pathman
	./pathman_test
	cd ../../utilities/pathman
elif [[ $BRANCH == "navx/"* ]]; then
	cd navx
	./navx_test
	cd ../../core/navigation/navx
elif [[ $BRANCH == "communication/"* ]]; then
	cd communication
	./communication_test
	cd ../../core/communication
elif [[ $BRANCH == "appendages/"* ]]; then
	cd appendages
	./appendages_test
	cd ../../appendages
fi

cppcheck src/*.cpp include/*.hpp
