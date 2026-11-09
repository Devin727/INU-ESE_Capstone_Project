#!/bin/bash
set -e
cd "$(dirname "$0")"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_PARTITION=${GZ_PARTITION:-co2_pipe_robot}
exec gz sim -v 3 "${1:-co2_pipe.sdf}"
