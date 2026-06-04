#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/install/setup.bash
exec /usr/bin/python3 /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cuda/gui/scripts/rtfg_launcher.py
