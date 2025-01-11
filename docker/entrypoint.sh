#!/bin/bash

# Set ULIMIT
ulimit -n 65535

# Reload ENV Variables
cd /root/trellis_ws

source $HOME/.bash_aliases

# Replace Bash & Inherit ENVs
echo "RUNNING $@"
exec "$@"
