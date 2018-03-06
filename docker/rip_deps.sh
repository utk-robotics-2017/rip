#!/bin/bash

pushd "$(dirname "$0")"
pushd ..

eval $(resize)

if command -v whiptail; then
  PROMPTER=whiptail
else
  if command -v dialog; then
    PROMPTER=dialog
  else
    echo "Please install whiptail or dialog first!"
    exit 1
  fi
fi

function git_branch_norm() {
  git symbolic-ref HEAD | cut -d'/' -f3- | sed -e 's;/;_;'
}

function prompt_docker_vtag() {
  DOCKER_VTAG=$($PROMPTER --title "rip_deps docker tag" \
      --menu "Choose the tag to use for utkrobotics/rip_deps:???" \
      $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 )) \
      "$DOCKER_VTAG" "Current set VTAG." \
      "latest" "utkrobotics/rip_deps:latest" \
      "$(git_branch_norm)" "Current git branch norm" \
      3>&1 1>&2 2>&3
  )
}

DOCKER_VTAG=${DOCKER_VTAG:-$(git_branch_norm)}
prompt_docker_vtag
echo "DOCKER_VTAG=$DOCKER_VTAG"

stdbuf -o0 \
docker build \
  -f docker/rip_deps.dockerfile \
  --tag utkrobotics/rip_deps:${DOCKER_VTAG} . | \
stdbuf -o0 grep -e '^Step' | \
stdbuf -o0 cut -d ' ' -f2,4- | \
stdbuf -o0 sed 's;/; ;1' | \
stdbuf -o0 awk '{pc=100*($1/$2);i=int(pc);print "XXX\n" i "\n" $0 "\nXXX"}' | \
$PROMPTER --title "Building rip_deps:${DOCKER_VTAG} container" --gauge "Starting Docker Build..." $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 ))

popd
popd

echo "Built utkrobotics/rip_deps:$DOCKER_VTAG"

