#!/bin/bash
colcon build --packages-up-to "$@" --event-handlers console_direct+
