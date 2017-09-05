# Checks if it compiles
make -j4

if [[ $2 ]]; then
	BRANCH = $1
else
	BRANCH = $3
fi

# Runs cpp check
if [[ $BRANCH == "arduino_gen/"* ]]; then
	./arduino_gen_test
	cd ../arduino_gen
else if [[ $BRANCH == "pathfinder/"* ]]; then
	./pathfinder_test
	cd ../core/navigation/pathfinder
else if [[ $BRANCH == "roboclaw/"* ]]; then
	./roboclaw_test
	cd ../utilities/roboclaw
else if [[ $BRANCH == "cmd_messenger/"* ]]; then
	./cmd_messenger_test
	cd ../utilities/cmd_messenger
else if [[ $BRANCH == "pathman/"* ]]; then
	./pathman_test
	cd ../utilities/pathman
else if [[ $BRANCH == "navx/"* ]]; then
	./navx_test
	cd ../core/navigation/navx
else if [[ $BRANCH == "communication/"* ]]; then
	./communication_test
	cd ../core/communication
else if [[ $BRANCH == "appendages/"* ]]; then
	./appendages_test
	cd ../appendages
fi

cppcheck src/*.cpp include/*.hpp