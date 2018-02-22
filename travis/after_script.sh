#!/usr/bin/env bash
set -e

if [[ $2 ]]; then
    BRANCH=$1
else
    BRANCH=$3
fi

bash <(curl -s https://codecov.io/bash) -t 233dc217-113d-4ab4-9db7-cfd9f85e41c7 \
    -f '!./#usr#*' \
    -f '!./external/**' \
    -f '!./#*#rip#external#**' \
    || echo "Codecov did not collect coverage reports"
