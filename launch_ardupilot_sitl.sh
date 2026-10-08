#!/bin/bash
export PATH=$PATH:$HOME/.local/bin

AUTOTEST_DIR="$HOME/ardupilot/ardupilot/Tools/autotest"

# Wipe stale EEPROM to force clean parameter load
rm -f eeprom.bin

echo "Launching ArduPilot SITL (Quad X + JSON backend)..."
~/ardupilot/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    -w \
    --model JSON \
    --add-param-file "$AUTOTEST_DIR/default_params/gazebo-iris.parm" \
    --console \
    --param="SCHED_LOOP_RATE=200" \
    --param="INS_ENABLE_CHECKS=0" \
    --param="ARMING_CHECK=0"
