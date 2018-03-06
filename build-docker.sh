#!/bin/bash

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

pushd "$(dirname "$0")"

eval $(resize)

function prompt_docker_vtag() {
  DOCKER_VTAG=$($PROMPTER --title "rip docker tag" \
      --menu "Choose the tag to use for utkrobotics/rip:???" \
      $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 )) \
      "$DOCKER_VTAG" "Current set DOCKER_VTAG." \
      "latest" "utkrobotics/rip:latest" \
      "$(git_branch_norm)" "Current git branch norm" \
      3>&1 1>&2 2>&3
  )
}

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
