#!/bin/bash

pushd "$(dirname "$0")"
pushd ..

source docker/core.sh

DOCKER_VTAG=${DOCKER_VTAG:-$(git_branch_norm)}
prompt_docker_vtag

if [[ "$DOCKER_STDOUT" == "raw" ]]; then
  docker  build \
  -f docker/rip_deps.dockerfile \
  --tag utkrobotics/rip_deps:${DOCKER_VTAG} .
else
  stdbuf -o0 \
  docker build \
    -f docker/rip_deps.dockerfile \
    --tag utkrobotics/rip_deps:${DOCKER_VTAG} . | \
  stdbuf -o0 grep -e '^Step' | \
  stdbuf -o0 cut -d ' ' -f2,4- | \
  stdbuf -o0 sed 's;/; ;1' | \
  stdbuf -o0 awk '{pc=100*($1/$2);i=int(pc);print "XXX\n" i "\n" $0 "\nXXX"}' | \
  $PROMPTER --title "Building rip_deps:${DOCKER_VTAG} container" --gauge "Starting Docker Build..." $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 ))
fi

popd
popd

echo "Built utkrobotics/rip_deps:$DOCKER_VTAG"

