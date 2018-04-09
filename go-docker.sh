#!/bin/zsh

set -E
SELFDIR="$(dirname $0 )"
pushd "$SELFDIR"

source docker/core.sh

# thing to execute in container
DEF_EXEC=("${SHELL}")

while [[ "$1" != "" ]] ; do
  case $1 in
    --rpi)
      RIPPROG=run_rpi
      ;;
    --prog)
      shift
      if [[ "$1" != "" ]]; then
        RIPPROG="$1"
      else
        echo " --prog requires a program selection to run."
        exit 1
      fi
      ;;
    --rpxc-tag)
      shift
      if [[ "$1" != "" ]]; then
        RPXC_IMAGE_TAG="$1"
        unset RPXC_IMAGE
        source docker/core.sh
      else
        echo " --rpxc-tag requires a docker tag id"
        exit 1
      fi
      ;;
    '--')
      shift
      if [[ "$1" != "" ]]; then
        DEF_EXEC=("$@")
      fi
      break
      ;;
    '-h'|'--help')
      cat << EOF
RIP Docker Script 'go-docker.sh'

Usage: ./go-docker.sh [options] [script args] [program ars ...]

Overridable Environment Variables:
  RPXC_IMAGE="utkrobotics/rip_rpi:latest"
          See also: --rpxc-tag flag
  RPXC_SCRIPT_LOCATION="./docker/rpi/rpxc/rpxc"
          Location of the RPXC script used to enter and setup the rip_rpi container from the host.
          Location relative to this shell script.
  RPXC_SYSROOT="/rpxc/sysroot"
          Location of sysroot *inside* the docker container.
          Do not change unless using a compatible/different docker image.
  RIPPROG
          Uses the same values as --prog flag does. (Shown in menu.)

  (for building containers...)
  RPI_DIST
          Sets the raspberry pi distro when building rip_rpi. (i.e. 'jessie'/'stretch')
  RPI_DOCKER_TAG
          Set the target docker tag for the rip_rpi build.

Options: <required> [optional]
  --help
          Display this help and exit.
  --prog <program selection>
          Select a program directly without being prompted by whiptail/dialog.
  --rpi [--] [command to run in container]
          Run the RPI emulation container directly with an optional command instead of a shell.
  --rpxc-tag <tag name>
          Specify an alternate tag (different from latest) to use for the docker repo.
          Used like 'utkrobotics/rip_rpi:<tag_name>'
  -- [anything else]
          Ignore flags after the '--' and pass them to the program selection.
EOF
      exit 0
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
  "build_stack" "Build the full RIP docker stack." \
  "cancel" "Do nothing." \
  3>&1 1>&2 2>&3
)
fi

function build_rip_deps() {
  echo "Building rip_deps container..."
  DOCKER_VTAG=latest ./docker/rip_deps.sh
}

function build_rip_rpi() {
  if ($PROMPTER --title "Comfirm Build?" --yesno "Do not run this command unless you were told to do so. Requires files outside of RIP." 8 68) then
    if find ../rpi_images -prune -empty ; then
      rpi_image_file=${rpi_image_file:-"$(find ../rpi_images -mindepth 1 -maxdepth 1 -printf '%f\n' | sort -g | tail -1 )"}
      echo "Found rpi debootstrap image ${rpi_image_file}"
      rsync --copy-links --times ../rpi_images/${rpi_image_file} ./external/rpi_images/
      if [ -z "$RPI_DOCKER_TAG" ]; then
        RPI_DOCKER_TAG=$($PROMPTER --title "Docker tag" \
          --inputbox "Choose docker tag for building image ${rpi_image_file}" \
          $(( LINES - 4 )) $(( COLUMNS - 18 )) \
          "latest" \
          3>&1 1>&2 2>&3
        )
        if [[ "$?" != "0" ]]; then
          echo "Cancelled."
          exit 0
        fi
      fi
      if [ -z "$RPI_DIST" ]; then
        RPI_DIST=$($PROMPTER --title "Distro release" \
          --inputbox "Enter distro name of image ${rpi_image_file}" \
          $(( LINES - 4 )) $(( COLUMNS - 18 )) \
          "stretch" \
          3>&1 1>&2 2>&3
        )
        if [[ "$?" != "0" ]]; then
          echo "Cancelled."
          exit 0
        fi
      fi
      echo "Building the rip_rpi container... this will take awhile."
      docker build -f docker/rip_rpi.dockerfile -t utkrobotics/rip_rpi:${RPI_DOCKER_TAG} --build-arg pi_image=${rpi_image_file} --build-arg pi_image_dist=${RPI_DIST} .
    else
      echo "No Raspbian debootstrap images could be located at ../rpi_images."
      exit 1
    fi
  else
    echo "Run me again to select another option."
  fi
}

function build_stack() {
  docker pull robobenklein/home
  build_rip_deps
  build_rip_rpi
}

function run_rpi() {
  # go to the user's working dir and run the container there.
  ABSPATH_RPXC="$(readlink -f $RPXC_SCRIPT_LOCATION )"
  popd
  "${ABSPATH_RPXC}" --image "$RPXC_IMAGE" \
    --args "--rm \
      --tmpfs=${RPXC_SYSROOT}/dev:rw,dev \
      --mount type=tmpfs,dst=${RPXC_SYSROOT}/dev/shm \
      --mount type=bind,src=/dev/pts,dst=/dev/pts \
      --mount type=bind,src=/dev/pts,dst=${RPXC_SYSROOT}/dev/pts \
      " -- ${DEF_EXEC}
  pushd "$SELFDIR"
}

if [ -n "$RIPPROG" ]; then
echo "Container command: ‘${(j: Ø :)DEF_EXEC}’"
echo "( $0 -- <cmd> <args> to change it )"
case $RIPPROG in
  run_rpi)
    run_rpi
    ;;
  new_rip)
    DOCKER_VTAG=${DOCKER_VTAG:-$(git_branch_norm)}
    if ! prompt_docker_vtag ; then
      # prompt cancelled
    else
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
    fi
    ;;
  update_rpi)
    docker pull $RPXC_IMAGE
    ;;
  build_rip_deps)
    build_rip_deps
    ;;
  build_rip_rpi)
    build_rip_rpi
    ;;
  build_stack)
    build_stack
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
