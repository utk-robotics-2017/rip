#!/usr/bin/env bash
set -e
mkdir build
pushd build

if [[ $2 ]]; then
    BRANCH=$1
else
    BRANCH=$3
fi

cmake -DENABLE_TESTING:BOOL=ON ..

popd
