#!/bin/bash
SRC_PATH=$1
rosdep install --from-paths $SRC_PATH --ignore-src -r -y
