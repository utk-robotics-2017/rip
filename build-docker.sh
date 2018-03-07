#!/bin/bash

set -E
pushd "$(dirname "$0")"

source docker/core.sh

$PROMPTER --title "Choose Action" \
  --menu "What do you want?" \
  $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 )) \
  "start_rpi" "Start a Raspberry Pi emulation container." \
  "start_rip"

DOCKER_VTAG=${DOCKER_VTAG:-$(git_branch_norm)}
prompt_docker_vtag
echo "DOCKER_VTAG=$DOCKER_VTAG"

if (! $PROMPTER --title "Build rip_deps?" \
      --yesno "Build the dependency container?" 8 68 \
      --yes-button "No" --no-button "Yes" \
) then
  echo "Building rip_deps container..."
  DOCKER_VTAG=latest ./docker/rip_deps.sh
fi

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

popd
set +E
