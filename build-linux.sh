#!/bin/zsh
set -E
pushd "$(dirname "$0")"

case "$1" in
  "--clean")
    if [ -d build ]; then
      echo "Removing $(dirname "$0")/build/"
      rm -r $(dirname "$0")/build/
    else
      echo "No $(dirname "$0")/build/ to remove."
    fi
    shift
    ;;
  "--help")
    echo "$0 [--clean|--help] <extra args are passed to cmake>"
    popd
    exit 0
    ;;
esac

mkdir -p build
pushd build

cleanup() {
  popd;popd;
  exit $SIGNAL;
}

trap 'SIGNAL=$?;cleanup' ERR
trap 'cleanup' SIGINT

cmake .. \
  -DENABLE_TESTING=on \
  $@
#

# test if system has more than 1GB of RAM
if (( $(cat /proc/meminfo | grep MemTotal | cut -d ' ' -f2- | tr -d ' [A-Za-z]' ) /1000 /1000 )); then
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

