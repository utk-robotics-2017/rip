#!/bin/zsh
set -E

BUILDDIR="build"
ENABLE_TESTING="ON"

if [[ "$(dpkg --print-architecture)" == "armhf" ]]; then
  BUILDDIR="${BUILDDIR}_armhf"
fi

while [[ "$1" != "" ]]; do
  case "$1" in
    "--")
      # stop parsing args and let cmake take the rest
      shift
      break
      ;;
    "--clean"|"-c")
      if [ -d "$BUILDDIR" ]; then
        echo "Removing $(pwd)/${BUILDDIR}/"
        rm -r "$(pwd)/${BUILDDIR}/"
      else
        echo "No $(pwd)/${BUILDDIR}/ to remove."
      fi
      ;;
    "--no-cd"|"-n")
      echo "Building project: $(pwd)"
      NO_CD="true"
      ;;
    "--production"|"-p")
      echo "Building production executables."
      echo " -DENABLE_TESTING=OFF"
      ENABLE_TESTING="OFF"
      ;;
    "-h"|"--help")
      cat << EOF
RIP build script for linux

usage: ./build-linux.sh [options] [--] [extra args passed to cmake]

Creates the dir 'build/' / 'build_armhf/'.
Runs Cmake with whatever extra args you gave it.
Attempts a multithreaded build.
Runs a single-threaded build if the multithreaded is unsuccessful.

Overridable Environment Variables:
  NO_CD
          Set this variable to prevent this script from changing back to the RIP directory.

Options:
  --help
          Display this message and exit.
  -c | --clean
          Remove the build folder before starting the build.
  -n | --no-cd
          Do not CD into this scripts' own directory when building.
          (Perform the same build steps on the current directory.)
          Used to build robot projects and others which depend on RIP.
  -p | --production
          Sets the Cmake variable ENABLE_TESTING to 'OFF'
          (Default is 'ON')
          Add this flag if you are building executables to copy to a Pi.
EOF
      exit 0
      ;;
  esac
  shift
done

if [ -z "$NO_CD" ]; then
  # push into own dir
  pushd "$(dirname "$0" )"
fi

mkdir -p $BUILDDIR
pushd $BUILDDIR

cleanup() {
  exit $SIGNAL;
}

trap 'SIGNAL=$?;cleanup' ERR
trap 'cleanup' SIGINT

cmake .. \
  -DENABLE_TESTING=${ENABLE_TESTING} \
  "$@"
#

# test if system has more than 1GB of RAM
if [[ ! -a /proc/meminfo ]] || (( $(cat /proc/meminfo | grep MemTotal | cut -d ' ' -f2- | tr -d ' [A-Za-z]' ) /1000 /1000 )); then
  nprocs="$(nproc --ignore=1)"
else
  # system does not have enough RAM for more than 1 thread :<
  nprocs="1"
fi
echo "Building with $nprocs threads."

trap '' ERR
make_retval=1
make -j$nprocs
make_retval=$?
trap 'SIGNAL=$?;cleanup' ERR

if [ $make_retval -eq 0 ]; then
  echo "Make completed."
elif [ $nprocs -gt 1 ]; then
  echo -e "\033[0;31m !!! Multithreaded make has failed, building single-threaded for error isolation. \033[0m"
  make
else
  echo -e "\033[0;31m !!! Compilation failed. \033[0m"
fi

