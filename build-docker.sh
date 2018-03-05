#!/bin/bash

if command -v whiptail; then
  echo "Preparing Docker Build..."
else
  echo "Please install whiptail first!"
  exit 1
fi

function git_branch_norm() {
  git symbolic-ref HEAD | cut -d'/' -f3- | sed -e 's;/;_;'
}

pushd "$(dirname "$0")"

stdbuf -o0 docker build --tag utkrobotics/rip:$(git_branch_norm) . | \
  stdbuf -o0 grep -e '^Step' | \
  stdbuf -o0 cut -d ' ' -f2,4- | \
  stdbuf -o0 sed 's;/; ;1' | \
  stdbuf -o0 awk '{pc=100*($1/$2);i=int(pc);print "XXX\n" i "\n" $0 "\nXXX"}' | \
  whiptail --title "Building RIP Docker container" --gauge "Starting Docker Build..." 6 50 0

if (whiptail --title "Start container?" --yesno "Start an interactive session in the docker container?" 8 68) then
  echo "    Running docker container for branch $(git_branch_norm)"
  docker run --rm -t -i utkrobotics/rip:$(git_branch_norm) $(basename $SHELL)
else
  echo "    To run the container:"
  echo -e "            \033[0;96mdocker run --rm -t -i utkrobotics/rip:$(git_branch_norm) $(basename $SHELL)\033[0;0m"
fi

popd
