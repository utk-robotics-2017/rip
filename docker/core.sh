#!/bin/bash

if command -v whiptail >/dev/null 2>&1; then
  PROMPTER=whiptail
else
  if command -v dialog; then
    PROMPTER=dialog
  else
    echo "Please install whiptail or dialog first!"
    exit 1
  fi
fi

RPXC_SCRIPT_LOCATION=${RPXC_SCRIPT_LOCATION:-'./docker/rpi/rpxc/rpxc'}
RPXC_IMAGE=${RPXC_IMAGE:-'utkrobotics/rip_rpi:latest'}
RPXC_SYSROOT=${RPXC_SYSROOT:-'/rpxc/sysroot'}

function git_branch_norm() {
  git symbolic-ref HEAD | cut -d'/' -f3- | sed -e 's;/;_;'
}

eval $(resize)

function prompt_docker_vtag() { # "title" "description"
  DOCKER_VTAG=$($PROMPTER --title "$1" \
      --menu "$2" \
      $(( LINES - 4 )) $(( COLUMNS - 18 )) $(( $LINES - 12 )) \
      "$DOCKER_VTAG" "Current environment's DOCKER_VTAG variable." \
      "latest" "Docker's default tag for images." \
      "$(git_branch_norm)" "Use current git branch as tag." \
      3>&1 1>&2 2>&3
  )
  echo "DOCKER_VTAG=$DOCKER_VTAG"
}

