#!/usr/bin/env bash

cd /Robot/CurrentArduinoCode/ttyACM0

git add -A
git commit -m "New code for ttyACM0"
git push

pio run -t upload
