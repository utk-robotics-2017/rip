#!/usr/bin/env bash

pushd external/g2o
mkdir build
cd build
cmake ..
make -j4
make install
popd
