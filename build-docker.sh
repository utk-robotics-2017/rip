#!/bin/bash

set -E
pushd "$(dirname "$0")"

source docker/core.sh

RIPPROG=$($PROMPTER --title "Choose Action" \
  --menu "What do you want?" \
  $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 )) \
  "run_rpi" "Start a Raspberry Pi emulation container mounted to your working dir." \
  "new_rip" "Build and run a new RIP container with the current local RIP codebase." \
  "build_rip_deps" "Build the rip_deps base dependency image." \
  "build_rip_rpi" "Build the rip_rpi Pi emulator and dependency container." \
  "cancel" "Do nothing." \
  3>&1 1>&2 2>&3
)

case $RIPPROG in
  run_rpi)
    # go to the user's working dir and run the container there.
    ABSPATH_RPXC=$(readlink -f $RPXC_SCRIPT_LOCATION )
    popd
    $ABSPATH_RPXC --image "$RPXC_IMAGE" \
      --args "--rm \
        --mount type=tmpfs,dst=${RPXC_SYSROOT}/dev,rw,dev \
        --mount type=devpts,dst=${RPXC_SYSROOT}/dev/pts \
        --mount type=tmpfs,dst=${RPXC_SYSROOT}/dev/shm \
      " -- \
      $(basename $SHELL )
    pushd "$(dirname "$0")"
    ;;
  new_rip)

    DOCKER_VTAG=${DOCKER_VTAG:-$(git_branch_norm)}
    prompt_docker_vtag
    echo "DOCKER_VTAG=$DOCKER_VTAG"
    stdbuf -o0 \
    docker build \
      --tag utkrobotics/rip:${DOCKER_VTAG} \
      -f docker/rip.dockerfile . | \
    stdbuf -o0 grep -e '^Step' | \
    stdbuf -o0 cut -d ' ' -f2,4- | \
    stdbuf -o0 sed 's;/; ;1' | \
    stdbuf -o0 awk '{pc=100*($1/$2);i=int(pc);print "XXX\n" i "\n" $0 "\nXXX"}' | \
    $PROMPTER --title "Building rip:${DOCKER_VTAG} container" --gauge "Starting Docker Build..." $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 ))
    
    if ($PROMPTER --title "Start container?" --yesno "Start an interactive session in the docker container?" 8 68) then
      echo "    Running docker container for branch $(git_branch_norm)"
      docker run --rm -t -i utkrobotics/rip:$(git_branch_norm) $(basename $SHELL)
    else
      echo "    To run the container:"
      echo -e "            \033[0;96mdocker run --rm -t -i utkrobotics/rip:$(git_branch_norm) $(basename $SHELL)\033[0;0m"
    fi

    ;;
  cancel)
    echo "I'm a very busy shell script, next time try not to waste my time? Thanks!"
    ;;
  *)
    echo -e "\033[0;31m Option not implemented!\033[0;0m"
esac

  echo "Building rip_deps container..."
  DOCKER_VTAG=latest ./docker/rip_deps.sh

popd
set +E
