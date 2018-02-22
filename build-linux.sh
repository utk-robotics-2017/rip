#!/bin/bash
pushd "$(dirname "$0")"

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

nprocs="$(nproc --ignore=1)"

trap '' ERR
make_retval=1
make -j$nprocs
make_retval=$?
trap 'SIGNAL=$?;cleanup' ERR

if [ $make_retval -eq 0 ]; then
  echo "Make completed."
else
  echo -e "\033[0;31m !!! Multithreaded make has failed, building single-threaded for error isolation. \033[0m"
  make
fi
