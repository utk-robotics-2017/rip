#!/bin/zsh
set -E

while [[ "$1" != "" ]]; do
  case "$1" in
    "--help"|"-h")
      echo "$0 <script args> <extra args are passed to cmake>"
      popd
      exit 0
      ;;
    "--")
      # stop parsing args and let cmake take the rest
      shift
      break
      ;;
    "--clean"|"-c")
      if [ -d build ]; then
        echo "Removing $(pwd)/build/"
        rm -r "$(pwd)/build/"
      else
        echo "No $(pwd)/build/ to remove."
      fi
      ;;
    "--no-cd"|"-n")
      echo "Building project: $(pwd)"
      NO_CD="true"
      ;;
  esac
  shift
done

if [ -z "$NO_CD" ]; then
  # push into own dir
  pushd "$(dirname "$0" )"
fi

mkdir -p build
pushd build

cleanup() {
  exit $SIGNAL;
}

trap 'SIGNAL=$?;cleanup' ERR
trap 'cleanup' SIGINT

cmake .. \
  -DENABLE_TESTING=on \
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

