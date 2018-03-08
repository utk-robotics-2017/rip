#!/bin/zsh

set -E
SELFDIR="$(dirname "$0")"
pushd $SELFDIR

source docker/core.sh

# thing to execute in container
DEF_EXEC="$(basename $SHELL )"

while [[ "$1" != "" ]] ; do
  case $1 in
    --rpi)
      RIPPROG=run_rpi
      ;;
    --)
      shift
      if [[ "$1" != "" ]]; then
        DEF_EXEC="$@"
      fi
      break
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
  shift
done

if [ -z "$RIPPROG" ]; then
RIPPROG=$($PROMPTER --title "Choose Action" \
  --menu "What do you want?" \
  $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 )) \
  "run_rpi" "Start a Raspberry Pi emulation container mounted to your working dir." \
  "new_rip" "Build and run a new RIP container with the current local RIP codebase." \
  "build_rip_deps" "Build the rip_deps base dependency image." \
  "update_rpi" "Pull latest rip_rpi image from the dockerhub." \
  "build_rip_rpi" "Build the rip_rpi Pi emulator and dependency container." \
  "cancel" "Do nothing." \
  3>&1 1>&2 2>&3
)
fi

if [ -n "$RIPPROG" ]; then
echo "Container command: ‘$DEF_EXEC’"
echo "( $0 -- <cmd> <args> to change it )"
case $RIPPROG in
  run_rpi)
    # go to the user's working dir and run the container there.
    ABSPATH_RPXC=$(readlink -f $RPXC_SCRIPT_LOCATION )
    popd
    $ABSPATH_RPXC --image "$RPXC_IMAGE" \
      --args "--rm \
        --tmpfs=${RPXC_SYSROOT}/dev:rw,dev \
        --mount type=tmpfs,dst=${RPXC_SYSROOT}/dev/shm \
        --mount type=bind,src=/dev/pts,dst=/dev/pts \
        --mount type=bind,src=/dev/pts,dst=${RPXC_SYSROOT}/dev/pts \
      " -- $DEF_EXEC
    pushd $SELFDIR
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
      docker run --rm -t -i utkrobotics/rip:$(git_branch_norm) $DEF_EXEC
    else
      echo "    To run the container:"
      echo -e "            \033[0;96mdocker run --rm -t -i utkrobotics/rip:$(git_branch_norm) $DEF_EXEC\033[0;0m"
    fi

    ;;
  update_rpi)
    docker pull $RPXC_IMAGE
    ;;
  build_rip_deps)
    echo "Building rip_deps container..."
    DOCKER_VTAG=latest ./docker/rip_deps.sh
    ;;
  cancel)
    echo "I'm a very busy shell script, next time try not to waste my time? Thanks!"
    ;;
  *)
    echo -e "\033[0;31m Option not implemented: $RIPPROG!\033[0;0m"
esac
fi

popd
set +E
